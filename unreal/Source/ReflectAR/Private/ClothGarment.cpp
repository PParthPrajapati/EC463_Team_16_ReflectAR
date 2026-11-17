#include "ClothGarment.h"
#include "GarmentFit.h"
#include "Async/Async.h"

namespace ReflectAR
{
	static constexpr int MaxVerts = 9000;

	// ---------------------------------------------------------------- bones
	/** Frame with Y along axisY and Z toward refZ (Unity: TRS(origin, LookRotation(z, y))). */
	static FFrame Frame(const V3& Origin, const V3& AxisY, const V3& RefZ)
	{
		const V3 y = Normalized(AxisY);
		V3 z = ProjectOnPlane(RefZ, y);
		if (z.SizeSquared() < 1e-6f) z = ProjectOnPlane(VFwd, y);
		return FFrame::Look(Origin, Normalized(z), y);
	}

	static void RestFramesOf(const FRestSkeleton& s, FFrame Out[6])
	{
		const V3 dl = FRestSkeleton::ArmDir(-1), dr = FRestSkeleton::ArmDir(1);
		const FFrame UpL = Frame(s.lSh, dl, VFwd), UpR = Frame(s.rSh, dr, VFwd);
		Out[0] = Frame(s.hipC, VUp, VFwd);
		Out[1] = Frame(s.shC, VUp, VFwd);
		Out[2] = UpL; Out[3] = UpR;
		Out[4] = Frame(s.lEl, dl, UpL.z);
		Out[5] = Frame(s.rEl, dr, UpR.z);
	}

	// ---------------------------------------------------------------- (re)build
	// Generating a garment takes 100-250 ms, so the mesh and its constraints are built on a worker
	// thread; only the swap happens on the game thread.

	bool FClothGarment::Drifted() const
	{
		auto D = [](float a, float b) { return FMath::Abs(a - b) / FMath::Max(b, 1e-3f); };
		if (D(Body.ShoulderW, Built.sw) > 0.25f || D(Body.TorsoLen, Built.L) > 0.25f
			|| D(Body.UpperL, Built.upperL) > 0.3f || D(Body.UpperR, Built.upperR) > 0.3f
			|| D(Body.ForeL, Built.foreL) > 0.3f || D(Body.ForeR, Built.foreR) > 0.3f) return true;
		for (int i = 0; i < 6; i++)
			if (FMath::Abs(Body.Profile[i] - Built.Profile[i]) > 0.08f || FMath::Abs(Body.DepthProfile[i] - Built.DepthProfile[i]) > 0.1f) return true;
		// the mould of the torso's front and the shoulders' slope: by more than a centimetre or so
		const FBodyMould& M = Body.Mould; const FBodyMould& W = Built.Mould;
		if (M.bValid != W.bValid || M.bShoulders != W.bShoulders) return true;
		for (int r = 0; M.bValid && r < FBodyMould::Rows; r++)
			for (int c = 0; c < FBodyMould::Cols; c++)
				if (FMath::Abs(M.Front[r][c] - W.Front[r][c]) > 0.012f) return true;
		for (int i = 0; M.bShoulders && i < FBodyMould::ShN; i++)
			if (FMath::Abs(M.Sh[i] - W.Sh[i]) > 0.012f) return true;
		return false;
	}

	void FClothGarment::Prepare(double Now)
	{
		if (!Body.Valid || Pending.IsValid()) return;
		bool bRebuild = !bHasMesh;
		if (bHasMesh)
		{
			// only a sustained change is worth a rebuild (it resets the cloth); tracking jitter is not
			const bool bDrift = Drifted();
			if (!bDrift) DriftSince = -1;
			else if (DriftSince < 0) DriftSince = Now;
			const double Cooldown = Now - FirstApplied < 15 ? 3 : 10;     // the measured profile settles first
			bRebuild = bDrift && Now - DriftSince > 1.5 && Now - LastApplied > Cooldown;
		}
		if (!bRebuild) return;
		const FRestSkeleton Skel = FRestSkeleton::From(Body);
		const FGarmentSpec* S = &Spec;
		// made from its authored mesh if the project has it, else generated from the cut
		TSharedPtr<const FGarmentAsset> Asset = Spec.Asset.IsEmpty() ? nullptr : FindGarmentAsset(Spec.Asset);
		Pending = Async(EAsyncExecution::ThreadPool, [Skel, S, Asset]() { return Generate(Skel, S, Asset); });
	}

	TSharedPtr<FClothGarment::FBuilt> FClothGarment::Generate(FRestSkeleton Skel, const FGarmentSpec* SpecPtr, TSharedPtr<const FGarmentAsset> Asset)
	{
		const FGarmentSpec& S = *SpecPtr;
		const double T0 = FPlatformTime::Seconds();
		// detail relative to the body, capped: solver cost grows with the vertex count
		const float Cell = GarmentMesher::DefaultCell * FMath::Max(Skel.u, 0.8f);
		TSharedPtr<FGarmentMeshData> D;
		if (Asset.IsValid()) D = FitGarmentAsset(*Asset, Skel, S.AssetEase);
		else
		{
			D = GarmentMesher::Build(Skel, S, Cell);
			if (D->Rest.Num() > MaxVerts)
				D = GarmentMesher::Build(Skel, S, Cell * FMath::Sqrt(D->Rest.Num() / (MaxVerts * 0.9f)));
		}
		// an authored mesh comes draped on a mannequin, but smoothly: without a little extra fabric it
		// fits the user like a moulded shell, with no folds at the elbows or down the body
		const float Slack = Asset.IsValid() ? FMath::Max(S.Slack, 0.03f) : S.Slack;
		TSharedPtr<FBuilt> R = MakeShared<FBuilt>();
		R->Skel = Skel; R->Data = D; R->bAuthored = Asset.IsValid();
		const int N = D->Rest.Num();
		R->MaxDist.SetNum(N); R->SkinK.SetNum(N);
		// Only the yoke, collar and sleeve caps are sewn to the body's motion. Below them the cloth is
		// free: it hangs from there and rests on the body. The leash is just a safety limit, and
		// elastic bands (ribbed hem, cuffs) hold on to the body.
		const float Leash = FMath::Min((0.07f + 2.f * S.Ease) * S.Drape, 0.2f) * Skel.u;
		for (int i = 0; i < N; i++)
		{
			const float l = D->Looseness[i], Band = D->Band[i];
			// Around the armholes and across the upper chest the cloth is cut close and many seams
			// meet: there it keeps to its cut within a couple of centimetres, or it slips into the
			// armpit and under the arm. It hangs free from the chest down and along the sleeves.
			const float High = Ramp(0.5f, 0.8f, D->Rest[i].Y / Skel.L);
			R->MaxDist[i] = Lerp(Lerp(0.002f, Leash * Lerp(1.f, 0.15f, High), Ramp(0.02f, 0.18f, l)), 0.012f * Skel.u, Band);
			R->SkinK[i] = FMath::Max3(Lerp(0.5f, 0.f, Ramp(0.f, 0.1f, l)), 0.2f * Band, 0.15f * High);
		}
		const TArray<int32>& Tris = D->Tris;
		TMap<int64, int32> EdgeSet;                                 // edge -> first opposite vertex
		TMap<int64, int32> EdgeIdx;                                 // edge -> its index in EA / EB
		TArray<uint8> Shared;                                       // per edge: used by two triangles
		auto Key = [](int32 a, int32 b) { return a < b ? (int64(a) << 32) | uint32(b) : (int64(b) << 32) | uint32(a); };
		for (int t = 0; t < Tris.Num(); t += 3)
			for (int e = 0; e < 3; e++)
			{
				const int a = Tris[t + e], b = Tris[t + (e + 1) % 3], Opp = Tris[t + (e + 2) % 3];
				const int64 K = Key(a, b);
				if (const int32* O = EdgeSet.Find(K)) { if (*O != Opp) { R->BA.Add(*O); R->BB.Add(Opp); } Shared[EdgeIdx[K]] = 1; }   // bending across the shared edge
				else { EdgeSet.Add(K, Opp); EdgeIdx.Add(K, R->EA.Num()); R->EA.Add(a); R->EB.Add(b); Shared.Add(0); }
			}
		R->ERest.SetNum(R->EA.Num()); R->BRest.SetNum(R->BA.Num());
		// Slack: the free fabric is a little larger than the shape it is held on, as a sewn garment
		// is, so it can't lie smooth and gathers into folds between the yoke, the bands and the body.
		auto Free = [&D](int a, int b)
		{
			// (sleeves stack into folds along the arm; the same extra on the body only reads as bagginess)
			float ArmW = 0.f;
			for (int k = 0; k < 4; k++) { if (D->Weights[a].Bone[k] >= 2) ArmW += 0.5f * D->Weights[a].W[k]; if (D->Weights[b].Bone[k] >= 2) ArmW += 0.5f * D->Weights[b].W[k]; }
			return Ramp(0.05f, 0.3f, FMath::Min(D->Looseness[a], D->Looseness[b])) * (1.f - FMath::Max(D->Band[a], D->Band[b])) * FMath::Lerp(0.4f, 1.f, Clamp01(ArmW));
		};
		for (int i = 0; i < R->EA.Num(); i++)
			R->ERest[i] = V3::Distance(D->Rest[R->EA[i]], D->Rest[R->EB[i]]) * (1.f + Slack * Free(R->EA[i], R->EB[i]));
		for (int i = 0; i < R->BA.Num(); i++)
			R->BRest[i] = V3::Distance(D->Rest[R->BA[i]], D->Rest[R->BB[i]]) * (1.f + Slack * Free(R->BA[i], R->BB[i]));
		// for shading: neighbours per vertex, and distance along the cloth from the nearest edge
		// of the garment (an edge used by one triangle), relaxed over the mesh edges
		float SumE = 0.f;
		for (float e : R->ERest) SumE += e;
		R->MeanEdge = R->EA.Num() > 0 ? SumE / R->EA.Num() : 0.02f;
		R->InvDeg.Init(0.f, N); R->EdgeDist.Init(EdgeBand, N);
		for (int i = 0; i < R->EA.Num(); i++)
		{
			R->InvDeg[R->EA[i]] += 1.f; R->InvDeg[R->EB[i]] += 1.f;
			if (!Shared[i]) R->EdgeDist[R->EA[i]] = R->EdgeDist[R->EB[i]] = 0.f;
		}
		for (float& d : R->InvDeg) d = d > 0.f ? 1.f / d : 0.f;
		for (int Pass = 0; Pass < 8; Pass++)
			for (int i = 0; i < R->EA.Num(); i++)
			{
				float& da = R->EdgeDist[R->EA[i]]; float& db = R->EdgeDist[R->EB[i]];
				da = FMath::Min(da, db + R->ERest[i]);
				db = FMath::Min(db, da + R->ERest[i]);
			}
		for (float& d : R->EdgeDist) d = Clamp01(d / EdgeBand);
		R->Ms = float((FPlatformTime::Seconds() - T0) * 1000.0);
		return R;
	}

	void FClothGarment::ApplyPending(double Now)
	{
		if (!Pending.IsValid() || !Pending.IsReady()) return;
		TSharedPtr<FBuilt> R = Pending.Get();
		Pending.Reset();
		if (!R.IsValid() || !R->Data.IsValid() || R->Data->Rest.Num() == 0) return;
		Built = R->Skel; Data = R->Data; bAuthored = R->bAuthored;
		MaxDist = MoveTemp(R->MaxDist); SkinK = MoveTemp(R->SkinK);
		EA = MoveTemp(R->EA); EB = MoveTemp(R->EB); BA = MoveTemp(R->BA); BB = MoveTemp(R->BB);
		ERest = MoveTemp(R->ERest); BRest = MoveTemp(R->BRest);
		EdgeDist = MoveTemp(R->EdgeDist); InvDeg = MoveTemp(R->InvDeg); MeanEdge = R->MeanEdge;
		const int N = Data->Rest.Num();
		X.SetNum(N); Prev.SetNum(N); Target.SetNum(N); LastTarget.SetNum(N); SimN.SetNum(N);
		Lap.SetNum(N); Ao.Init(1.f, N);

		const int Rn = Data->RenderToSim.Num();
		BodySection = FMeshSection();
		BodySection.Pos.SetNum(Rn); BodySection.Nrm.SetNum(Rn); BodySection.Tan.SetNum(Rn);
		BodySection.Col.Init(FColor::White, Rn);
		BodySection.UV = Data->UV;
		BodySection.Tris = Data->RenderTris;
		for (int i = 0; i < Rn; i++)
		{
			BodySection.Pos[i] = Data->Rest[Data->RenderToSim[i]];
			BodySection.Nrm[i] = Data->RestN[Data->RenderToSim[i]];
			BodySection.Tan[i] = V4(1, 0, 0, 1);
		}
		Trims = FGarmentTrims::Build(*Data, Built, Spec);
		const int Bn = Data->RibVerts.Num();
		RibSection = FMeshSection();
		RibSection.Pos.SetNumZeroed(Bn); RibSection.Nrm.SetNumZeroed(Bn); RibSection.Tan.Init(V4(1, 0, 0, 1), Bn);
		RibSection.UV = Data->RibUV;
		RibSection.Tris = Data->RibTris;
		RestFramesOf(Built, RestFrames);
		// the body's cross-section vs height (rest units), closing in a dome under the shoulder line
		float MinY = Spec.Hem * Built.L;
		for (const V3& p : Data->Rest) MinY = FMath::Min(MinY, p.Y);
		TorsoT0 = MinY / Built.L - 0.2f; TorsoT1 = 1.06f;
		for (int k = 0; k < TorsoRows; k++)
		{
			const float t = FMath::Lerp(TorsoT0, TorsoT1, k / float(TorsoRows - 1));
			// (starting well below it: the garment's own shoulders round off there, and are sewn on)
			const float Cap = Clamp01((t - 0.66f) / (TorsoT1 - 0.1f - 0.66f)), Dome = FMath::Sqrt(FMath::Max(1.f - Cap * Cap, 0.f));
			for (int c = 0; c < FBodyMould::Cols; c++)
			{
				Built.TorsoSection(t * Built.L, c * FBodyMould::ColStep, TorsoA[k], TorsoC[k], TorsoF[k][c], TorsoB[k]);
				TorsoF[k][c] *= Dome;
			}
			TorsoA[k] *= FMath::Lerp(1.f, Dome, 0.5f); TorsoB[k] *= Dome; TorsoC[k] *= Dome;
		}
		bHasMesh = true;
		bSimInit = false;
		bTopologyChanged = true;
		LastApplied = Now;
		if (RebuildCount == 0) FirstApplied = Now;
		DriftSince = -1;
		LastBuildMs = R->Ms;
		RebuildCount++;
	}

	void FClothGarment::SkinTargets()
	{
		const V3* J = Body.J;
		const V3 dUL = J[LM::LElbow] - J[LM::LShoulder], dUR = J[LM::RElbow] - J[LM::RShoulder];
		const FFrame UpL = Frame(J[LM::LShoulder], dUL, Body.Fwd), UpR = Frame(J[LM::RShoulder], dUR, Body.Fwd);
		const FFrame Cur[6] =
		{
			Body.PelvisFrame(), Body.ChestFrame(), UpL, UpR,
			Frame(J[LM::LElbow], J[LM::LWrist] - J[LM::LElbow], UpL.z),
			Frame(J[LM::RElbow], J[LM::RWrist] - J[LM::RElbow], UpR.z),
		};
		for (int b = 0; b < 6; b++) SkinM[b] = FAffine::Between(RestFrames[b], Cur[b]);
		const TArray<FBoneWeights>& W = Data->Weights; const TArray<V3>& R = Data->Rest;
		for (int i = 0; i < R.Num(); i++)
		{
			const FBoneWeights& Bw = W[i];
			Target[i] = SkinM[Bw.Bone[0]].Apply(R[i]) * Bw.W[0] + SkinM[Bw.Bone[1]].Apply(R[i]) * Bw.W[1]
				+ SkinM[Bw.Bone[2]].Apply(R[i]) * Bw.W[2] + SkinM[Bw.Bone[3]].Apply(R[i]) * Bw.W[3];
		}
	}

	/** Capsules approximating the actual body (no ease), for cloth collisions. */
	void FClothGarment::BuildColliders()
	{
		Colliders.Reset();
		const float u = Body.Unit();
		for (int k = 0; k < 5; k++)
		{
			const float y = FMath::Lerp(Spec.Hem - 0.1f, 0.92f, k / 4.f) * Body.TorsoLen;
			float a, d;
			Built.TorsoRadii(y, a, d);
			a *= Body.ShoulderW / Built.sw; d *= 0.95f;
			const float w = SmoothStep(0.f, 1.f, (y / Body.TorsoLen - 0.1f) / 0.7f);
			const V3 r = SlerpDir(Body.PRight, Body.Right, w);
			const V3 c = Body.HipC + Body.Up * y;
			const float Half = FMath::Max(a - d, 0.005f);
			Colliders.Emplace(c - r * Half, c + r * Half, d);
		}
		const V3* J = Body.J;
		Colliders.Emplace(J[LM::LShoulder], J[LM::RShoulder], (bAuthored ? 0.04f : 0.05f) * u);   // under the authored garments' sloped shoulders
		Colliders.Emplace(J[LM::LShoulder], J[LM::LElbow], 0.043f * u);
		Colliders.Emplace(J[LM::RShoulder], J[LM::RElbow], 0.043f * u);
		Colliders.Emplace(J[LM::LElbow], J[LM::LWrist], 0.036f * u);
		Colliders.Emplace(J[LM::RElbow], J[LM::RWrist], 0.036f * u);
		Colliders.Emplace(Body.ShC + Body.Up * 0.05f * u, Body.HeadC - Body.Up * 0.04f * u, 0.05f * u);   // neck, clear of the yoke
	}

	void FClothGarment::Outline(TArray<FCapsuleB>& Into) const
	{
		const float R = Room();
		for (int i = 0; i < 6 && i < Colliders.Num(); i++)      // 5 torso levels + the shoulder line
			Into.Emplace(Colliders[i].a, Colliders[i].b, Colliders[i].r + R);
	}

	// ---------------------------------------------------------------- simulation
	/** Pushes P out of the torso: an ellipse per height that twists from the pelvis to the chest. */
	bool FClothGarment::CollideTorso(V3& P, V3& OutN) const
	{
		const V3 Rel = P - Body.HipC;
		const float y = Dot(Rel, Body.Up);
		float f = (y / Body.TorsoLen - TorsoT0) / (TorsoT1 - TorsoT0) * (TorsoRows - 1);
		if (f < 0.f || f >= TorsoRows - 1) return false;
		const int k = int(f); f -= k;
		const float a = FMath::Lerp(TorsoA[k], TorsoA[k + 1], f) * TorsoScale + Thick;
		const V3 R = LerpU(TorsoR[k], TorsoR[k + 1], f), F = Cross(R, Body.Up);
		const float x = Dot(Rel, R), z = Dot(Rel, F) - FMath::Lerp(TorsoC[k], TorsoC[k + 1], f);
		// behind the section's centre the generic back; in front of it the measured surface there
		float d = FMath::Lerp(TorsoB[k], TorsoB[k + 1], f);
		if (z > 0.f)
		{
			const float fc = FMath::Abs(x) / (a * FBodyMould::ColStep);
			d = FMath::Lerp(FBodyMould::AcrossAt(TorsoF[k], fc), FBodyMould::AcrossAt(TorsoF[k + 1], fc), f);
		}
		d += Thick;
		if (d < 0.012f) return false;
		const float q2 = x * x / (a * a) + z * z / (d * d);
		if (q2 >= 1.f) return false;
		// one Newton step of q - 1 = 0 along its gradient (radial near the axis, where that vanishes)
		const float q = FMath::Sqrt(q2);
		float gx = x / (a * a), gz = z / (d * d);
		if (q < 0.25f) { gx = x + 1e-4f; gz = z + 1e-4f; }
		const float gl = FMath::Sqrt(gx * gx + gz * gz);
		OutN = (R * gx + F * gz) / gl;
		const float Push = q < 0.25f ? FMath::Min(a, d) * (1.f - q) : (1.f - q) * q / gl;
		P += OutN * FMath::Min(Push, 0.05f);
		return true;
	}

	void FClothGarment::Step(float Dt, double Now)
	{
		if (!Body.Valid) return;
		ApplyPending(Now);
		Prepare(Now);
		if (!bHasMesh) return;
		const double T0 = FPlatformTime::Seconds();
		SkinTargets();
		BuildColliders();
		const int N = X.Num();
		const bool bFresh = !bSimInit;
		if (!bSimInit) { X = Target; Prev = Target; LastTarget = Target; bSimInit = true; }
		// worn again after a while off: it comes back as it last hung, carried to the current pose
		const bool bResume = Now - LastStep > 0.25;
		LastStep = Now;
		// Carry the cloth along with the body's own motion: only (1 - Follow) of each movement is
		// left to inertia, so the garment doesn't trail behind the user but still swings locally.
		for (int i = 0; i < N; i++)
		{
			const V3 Moved = (Target[i] - LastTarget[i]) * (bResume ? 1.f : Follow);
			X[i] += Moved; Prev[i] = bResume ? X[i] : Prev[i] + Moved;
			LastTarget[i] = Target[i];
		}

		TorsoScale = Body.ShoulderW / Built.sw;
		for (int k = 0; k < TorsoRows; k++)
		{
			const float t = FMath::Lerp(TorsoT0, TorsoT1, k / float(TorsoRows - 1));
			TorsoR[k] = SlerpDir(Body.PRight, Body.Right, SmoothStep(0.f, 1.f, (t - 0.1f) / 0.7f));
		}

		Dt = FMath::Min(Dt, 1.f / 30.f);
		// a garment put on for the first time drops into place: settle it a little before it is shown
		const int Steps = bFresh ? Substeps * (bAuthored ? 3 : 10) : Substeps;   // an authored mesh starts out draped
		const float h = bFresh ? 1.f / 480.f : Dt / Substeps, h2 = h * h;
		const V3 G = V3(0.f, -9.81f, 0.f) * Spec.Density * h2;
		const float Keep = 1.f - Clamp01((bFresh ? 4.f : 1.f) * Spec.Damping * h * 60.f);
		const float AS = Spec.Stretch / h2, AC = Spec.Compress / h2, AB = Spec.Bend / h2;
		const int NC = Colliders.Num();
		ColCenter.SetNum(NC); ColBound.SetNum(NC);
		for (int c = 0; c < NC; c++)
		{
			ColCenter[c] = (Colliders[c].a + Colliders[c].b) * 0.5f;
			const float Rr = V3::Distance(Colliders[c].a, Colliders[c].b) * 0.5f + Colliders[c].r + Thick;
			ColBound[c] = Rr * Rr;
		}
		const float SkinRate = h * 60.f;

		for (int s = 0; s < Steps; s++)
		{
			// predict (Verlet with damping)
			for (int i = 0; i < N; i++)
			{
				const V3 V = (X[i] - Prev[i]) * Keep;
				Prev[i] = X[i];
				X[i] += V + G;
			}
			// stretch / compression (XPBD, one iteration per small substep). Compression is much
			// softer than stretch but not free, so slack buckles into folds instead of vanishing.
			for (int k = 0; k < EA.Num(); k++)
			{
				const int a = EA[k], b = EB[k];
				const V3 D = X[a] - X[b];
				const float Len = D.Size();
				if (Len < 1e-7f) continue;
				const float C = Len - ERest[k];
				const float Dl = -C / (2.f + (C > 0 ? AS : AC));
				const V3 Corr = D * (Dl / Len);
				X[a] += Corr; X[b] -= Corr;
			}
			// bending (distance across each pair of triangles)
			for (int k = 0; k < BA.Num(); k++)
			{
				const int a = BA[k], b = BB[k];
				const V3 D = X[a] - X[b];
				const float Len = D.Size();
				if (Len < 1e-7f) continue;
				const float Dl = -(Len - BRest[k]) / (2.f + AB);
				const V3 Corr = D * (Dl / Len);
				X[a] += Corr; X[b] -= Corr;
			}
			// sewn-on regions follow the body; the rest collides with it, on a leash
			for (int i = 0; i < N; i++)
			{
				V3 P = X[i];
				const V3 T = Target[i];
				if (SkinK[i] > 0.f) P = LerpU(P, T, 1.f - FMath::Pow(1.f - SkinK[i], SkinRate));
				const float M = MaxDist[i];
				if (M > 0.02f)                                   // the yoke is sewn on: nothing to collide
				{
					V3 Nrm;
					for (int c = FirstLimb; c < NC; c++)
					{
						if ((ColCenter[c] - P).SizeSquared() > ColBound[c]) continue;   // outside the capsule's bounding sphere
						const FCapsuleB& Cap = Colliders[c];
						Nrm = P - Cap.Closest(P);
						const float R = Cap.r + Thick, Dd = Nrm.SizeSquared();
						if (Dd < R * R && Dd > 1e-10f)
						{
							const float Dist = FMath::Sqrt(Dd);
							P += Nrm * ((R - Dist) / Dist);
							// friction: kill some tangential motion while in contact
							const V3 Vel = P - Prev[i];
							const V3 Vn = Nrm * (Dot(Vel, Nrm) / Dd);
							Prev[i] += (Vel - Vn) * Friction;
						}
					}
					// the torso last: where an arm presses into a wide body, the cloth stays outside the body
					if (CollideTorso(P, Nrm))
					{
						const V3 Vel = P - Prev[i];
						Prev[i] += (Vel - Nrm * Dot(Vel, Nrm)) * Friction;
					}
				}
				V3 Off = P - T;
				// Fabric barely stretches, but a few solver passes over a garment's whole length leave
				// it sagging like rubber: it may not hang lower than a little below where its cut
				// puts it. In and out, and sideways, it is free up to the leash.
				const float Down = Dot(Off, Body.Up);
				if (Down < -MaxSag) Off -= Body.Up * (Down + MaxSag);
				if (Off.SizeSquared() > M * M) Off = Normalized(Off) * M;
				X[i] = T + Off;
			}
		}
		UpdateRenderMesh();
		if (Trims.IsValid()) Trims->Update(X, SimN);
		LastStepMs = float((FPlatformTime::Seconds() - T0) * 1000.0);
	}

	void FClothGarment::UpdateRenderMesh()
	{
		// normals on the welded mesh, so UV seams don't show in the shading
		GarmentMesher::Normals(X, Data->Tris, SimN);
		const TArray<int32>& ToSim = Data->RenderToSim;
		Shade();
		for (int r = 0; r < ToSim.Num(); r++)
		{
			const int i = ToSim[r];
			BodySection.Pos[r] = X[i]; BodySection.Nrm[r] = SimN[i];
			BodySection.Col[r] = FColor(uint8(Ao[i] * 255.f + 0.5f), uint8(EdgeDist[i] * 255.f + 0.5f), 255, 255);
		}
		Tangents(BodySection);
		// ribbed bands: a layer just above the cloth
		constexpr float Lift = 0.0025f;
		for (int r = 0; r < Data->RibVerts.Num(); r++)
		{
			const FGarmentMeshData::FRibVert& V = Data->RibVerts[r];
			const V3 n = Normalized(LerpU(SimN[V.A], SimN[V.B], V.T));
			RibSection.Pos[r] = LerpU(X[V.A], X[V.B], V.T) + n * Lift;
			RibSection.Nrm[r] = n;
		}
		if (RibSection.Tris.Num() > 0) Tangents(RibSection);
	}

	/**
	 * How open each vertex is, for the material: darker in the valleys of folds (its neighbours lie
	 * on the side its normal points to) and where a body part is close in front of it, as under an
	 * arm. Eased over a few frames, so it doesn't flicker with the solver.
	 */
	void FClothGarment::Shade()
	{
		const int N = X.Num();
		if (Lap.Num() != N || Ao.Num() != N || InvDeg.Num() != N) return;
		for (V3& l : Lap) l = V3::ZeroVector;
		for (int k = 0; k < EA.Num(); k++)
		{
			const V3 D = X[EB[k]] - X[EA[k]];
			Lap[EA[k]] += D; Lap[EB[k]] -= D;
		}
		const float InvEdge = 1.f / FMath::Max(MeanEdge, 1e-3f);
		constexpr float Reach = 0.12f;
		for (int i = 0; i < N; i++)
		{
			const V3 P = X[i], Nn = SimN[i];
			float A = 1.f - FMath::Clamp(Dot(Lap[i], Nn) * InvDeg[i] * InvEdge * 1.6f, -0.06f, 0.35f);
			auto Occlude = [&](const FCapsuleB& C)
			{
				const V3 To = C.Closest(P) - P;
				const float D2 = To.SizeSquared(), Lim = C.r + Reach;
				if (D2 > Lim * Lim || D2 < 1e-8f) return;
				const float D = FMath::Sqrt(D2), Facing = Dot(Nn, To) / D;
				if (Facing <= 0.f) return;                                   // behind the cloth: the body part it lies on
				const float Solid = FMath::Min(C.r / D, 1.f);
				A *= 1.f - 0.65f * Facing * Solid * Solid * Clamp01((Lim - D) / Reach * 2.f);
			};
			for (int c = 0; c < Colliders.Num(); c++) if (c != FirstLimb) Occlude(Colliders[c]);   // torso levels, arms and neck (not the shoulder line under the cloth)
			Ao[i] += (FMath::Clamp(A, 0.3f, 1.f) - Ao[i]) * 0.35f;
		}
	}

	/** Per-vertex tangent along the fabric's u direction, with handedness. */
	void FClothGarment::Tangents(FMeshSection& Sec)
	{
		const TArray<V3>& P = Sec.Pos;
		TAcc.Init(V3::ZeroVector, P.Num());
		BAcc.Init(V3::ZeroVector, P.Num());
		const TArray<int32>& Rt = Sec.Tris; const TArray<V2>& UV = Sec.UV;
		for (int t = 0; t < Rt.Num(); t += 3)
		{
			const int i0 = Rt[t], i1 = Rt[t + 1], i2 = Rt[t + 2];
			const V3 E1 = P[i1] - P[i0], E2 = P[i2] - P[i0];
			const V2 D1 = UV[i1] - UV[i0], D2 = UV[i2] - UV[i0];
			const float Det = D1.X * D2.Y - D2.X * D1.Y;
			if (FMath::Abs(Det) < 1e-12f) continue;
			const float Inv = 1.f / Det;
			const V3 T = (E1 * D2.Y - E2 * D1.Y) * Inv, B = (E2 * D1.X - E1 * D2.X) * Inv;
			TAcc[i0] += T; TAcc[i1] += T; TAcc[i2] += T;
			BAcc[i0] += B; BAcc[i1] += B; BAcc[i2] += B;
		}
		for (int r = 0; r < P.Num(); r++)
		{
			const V3 n = Sec.Nrm[r];
			V3 T = TAcc[r] - n * Dot(n, TAcc[r]);
			if (T.SizeSquared() < 1e-12f) T = Cross(n, FMath::Abs(n.Y) < 0.9f ? VUp : VRight);
			T = Normalized(T);
			const float W = Dot(Cross(n, T), BAcc[r]) < 0.f ? -1.f : 1.f;
			Sec.Tan[r] = V4(T.X, T.Y, T.Z, W);
		}
	}
}
