#include "BodyRig.h"
#include "GarmentMesher.h"

namespace ReflectAR
{
	FBodyRig::FBodyRig()
	{
		for (V3& j : J) j = V3::ZeroVector;
		HipC = ShC = NeckBase = HeadC = V3::ZeroVector;
	}

	void FBodyRig::Update(const V3* Target, const float* InVis, float Dt, bool bSnap)
	{
		const float a = bSnap || Smoothing <= 0.f ? 1.f : 1.f - FMath::Exp(-Smoothing * Dt);
		bSnap = bSnap || !bMeasured;
		for (int i = 0; i < LM::Count; i++) Vis[i] = InVis[i];
		if (bStabilise) Stabilise(Target, Dt, bSnap);
		else
		{
			for (int i = 0; i < LM::Count; i++) J[i] = LerpV(J[i], Target[i], a);
			bFrontal = true;
			for (bool& b : bArmSquare) b = true;
		}
		Derive(bSnap);
		Valid = true;
	}

	// ------------------------------------------------------------------ depth stabilisation
	static const int Torso4[4] = { LM::LShoulder, LM::RShoulder, LM::LHip, LM::RHip };
	// joints that hang off each shoulder / hip and move in depth with it
	static const int ChainLSh[] = { LM::LElbow, LM::LWrist, LM::LPinky, LM::LIndex, LM::LThumb };
	static const int ChainRSh[] = { LM::RElbow, LM::RWrist, LM::RPinky, LM::RIndex, LM::RThumb };
	static const int ChainLHip[] = { LM::LKnee, LM::LAnkle, 29, 31 };
	static const int ChainRHip[] = { LM::RKnee, LM::RAnkle, 30, 32 };

	/**
	 * Depth difference between joints a and b (distance from the camera, b minus a), given how
	 * far apart they really are. Their image positions fix how wide the pair looks; whatever of
	 * Width is missing must be depth. Near square-on that is ill-conditioned (and Width is only
	 * known to a few percent), so the tracked difference is used there and blended out as the
	 * pair turns. Updates Rho[a], Rho[b] about their mean and returns the change applied to b.
	 */
	float FBodyRig::SolvePair(int a, int b, int Which, float Width, const V3* Dir, float* Rho)
	{
		const float Mid = 0.5f * (Rho[a] + Rho[b]), Cs = FMath::Clamp(Dot(Dir[a], Dir[b]), -1.f, 1.f);
		const float Seen2 = 2.f * Mid * Mid * (1.f - Cs);                 // squared distance if both were at the same depth
		const float Fore = FMath::Sqrt(FMath::Max(Width * Width - Seen2, 0.f) * 2.f / (1.f + Cs));
		const float Tracked = Rho[b] - Rho[a];
		// which one is nearer: from the tracker, but it may only change its mind while the pair is
		// close to square-on (a real turn passes through there) or when it is emphatic
		float& Sign = PairSign[Which];
		const float Want = Tracked >= 0.f ? 1.f : -1.f;
		if (Sign == 0.f || (Want != Sign && (Fore < 0.45f * Width || FMath::Abs(Tracked) > 0.25f * Width))) Sign = Want;
		// (an estimated depth is flattened from the first degrees of a turn; a measured one only fails
		// once one joint hides the other. On a D415 recording the shoulder joints sat 17 % closer
		// together seen from behind than from the front, which read as 35 degrees of turn.)
		const float From = bDepthMeasured ? 0.62f : 0.28f, Over = bDepthMeasured ? 0.25f : 0.22f;
		const float w = SmoothStep(0.f, 1.f, (Fore / FMath::Max(Width, 1e-3f) - From) / Over);
		const float Delta = Tracked + (Sign * Fore - Tracked) * w;
		const float Shift = (Mid + 0.5f * Delta) - Rho[b];
		Rho[a] = Mid - 0.5f * Delta;
		Rho[b] = Mid + 0.5f * Delta;
		return Shift;
	}

	void FBodyRig::Stabilise(const V3* Target, float Dt, bool bSnap)
	{
		V3 Dir[LM::Count];
		float Rho[LM::Count];
		for (int i = 0; i < LM::Count; i++)
		{
			const V3 d = Target[i] - CamPos;
			Rho[i] = FMath::Max(d.Size(), 1e-3f);
			Dir[i] = d / Rho[i];
		}
		float Root = 0.f;
		for (int i : Torso4) Root += Rho[i] * 0.25f;

		// ---- One-Euro on each joint's depth relative to the torso
		const float SafeDt = FMath::Clamp(Dt, 1e-3f, 0.1f);
		auto Alpha = [SafeDt](float Cutoff) { return 1.f / (1.f + 1.f / (2.f * PI * Cutoff * SafeDt)); };
		for (int i = 0; i < LM::Count; i++)
		{
			const float Rel = Rho[i] - Root;
			if (bSnap) { RelDepth[i] = RelRaw[i] = Rel; RelVel[i] = 0.f; }
			else
			{
				RelVel[i] += ((Rel - RelRaw[i]) / SafeDt - RelVel[i]) * Alpha(1.f);
				RelRaw[i] = Rel;
				RelDepth[i] += (Rel - RelDepth[i]) * Alpha(DepthMinCutoff + DepthBeta * FMath::Abs(RelVel[i]));
			}
			Rho[i] = Root + RelDepth[i];
		}

		// ---- can lengths be measured right now? only what is seen (nearly) side-on
		auto Square = [&](int a, int b)
		{
			const float Mid = 0.5f * (Rho[a] + Rho[b]);
			const float Seen = Mid * FMath::Sqrt(FMath::Max(2.f * (1.f - Dot(Dir[a], Dir[b])), 0.f));
			return FMath::Abs(Rho[b] - Rho[a]) < 0.2f * Seen && Vis[a] > 0.5f && Vis[b] > 0.5f;
		};
		bFrontal = Square(LM::LShoulder, LM::RShoulder);
		bArmSquare[0] = Square(LM::LShoulder, LM::LElbow); bArmSquare[1] = Square(LM::RShoulder, LM::RElbow);
		bArmSquare[2] = Square(LM::LElbow, LM::LWrist); bArmSquare[3] = Square(LM::RElbow, LM::RWrist);

		// ---- shoulders and hips: depth difference from foreshortening
		if (!bSnap)
		{
			const float dSh = SolvePair(LM::LShoulder, LM::RShoulder, 0, ShoulderW, Dir, Rho);
			for (int i : ChainRSh) Rho[i] += dSh;
			for (int i : ChainLSh) Rho[i] -= dSh;
			const float dHip = SolvePair(LM::LHip, LM::RHip, 1, HipW, Dir, Rho);
			for (int i : ChainRHip) Rho[i] += dHip;
			for (int i : ChainLHip) Rho[i] -= dHip;
		}
		else PairSign[0] = PairSign[1] = 0.f;

		for (int i = 0; i < LM::Count; i++) J[i] = CamPos + Dir[i] * Rho[i];
	}

	void FBodyRig::Derive(bool bResetMeasures)
	{
		HipC = (J[LM::LHip] + J[LM::RHip]) * 0.5f;
		ShC = (J[LM::LShoulder] + J[LM::RShoulder]) * 0.5f;
		Up = Normalized(ShC - HipC);
		Right = Normalized(ProjectOnPlane(J[LM::RShoulder] - J[LM::LShoulder], Up));
		Fwd = Cross(Right, Up);
		// the pelvis turns with the chest, give or take a twist at the waist; its own measurement is
		// poor (the hip joints are close together and hidden), so it only gets half a say
		V3 Pr = Normalized(ProjectOnPlane(J[LM::RHip] - J[LM::LHip], Up));
		if (Pr.SizeSquared() < 0.5f || Dot(Pr, Right) < 0.f) Pr = Right;
		PRight = SlerpDir(Right, Pr, 0.5f);
		PFwd = Cross(PRight, Up);
		const V3 ToCam = Normalized(ProjectOnPlane(CamPos - ShC, Up));
		YawDeg = FMath::RadiansToDegrees(FMath::Atan2(Dot(Cross(Fwd, ToCam), Up), Dot(Fwd, ToCam)));

		// Body measurements change slowly; smooth hard so garments don't breathe. Turned away from
		// the camera a length can only read short (foreshortened, and the tracker flattens depth),
		// so a longer reading is believed readily and a shorter one only slowly, while seen side-on.
		const float TorsoNow = FMath::Clamp(FVector3f::Distance(HipC, ShC), 0.35f, 0.75f);
		TorsoLen = Lerp(TorsoLen, TorsoNow, bResetMeasures ? 1.f : 0.03f);
		auto Measure = [bResetMeasures](float& Len, float Now, float Lo, float Hi, float Prior, bool bSquare)
		{
			Now = FMath::Clamp(Now, Lo, Hi);
			if (bResetMeasures) Len = FMath::Max(Now, FMath::Min(Prior, Hi));   // first sight may be foreshortened
			else if (Now > Len) Len = Lerp(Len, Now, 0.05f);
			else if (bSquare) Len = Lerp(Len, Now, 0.004f);
		};
		Measure(ShoulderW, Seg(LM::LShoulder, LM::RShoulder), 0.25f, 0.55f, 0.62f * TorsoNow, bFrontal);
		Measure(HipW, Seg(LM::LHip, LM::RHip), 0.15f, 0.40f, 0.36f * TorsoNow, bFrontal);
		Measure(HeadR, Seg(LM::LEar, LM::REar) * 0.62f, 0.075f, 0.13f, 0.f, bFrontal);
		Measure(UpperL, Seg(LM::LShoulder, LM::LElbow), 0.18f, 0.40f, 0.5f * TorsoNow, bArmSquare[0]);
		Measure(UpperR, Seg(LM::RShoulder, LM::RElbow), 0.18f, 0.40f, 0.5f * TorsoNow, bArmSquare[1]);
		Measure(ForeL, Seg(LM::LElbow, LM::LWrist), 0.16f, 0.36f, 0.45f * TorsoNow, bArmSquare[2]);
		Measure(ForeR, Seg(LM::RElbow, LM::RWrist), 0.16f, 0.36f, 0.45f * TorsoNow, bArmSquare[3]);
		ArmLenL = UpperL + ForeL;
		ArmLenR = UpperR + ForeR;
		bMeasured = true;

		NeckBase = ShC + (Up * 0.03f - Fwd * 0.01f) * Unit();
		HeadC = (J[LM::LEar] + J[LM::REar]) * 0.5f;
	}

	// ------------------------------------------------------------------ body shape
	void FBodyShape::Reset(FBodyRig& B)
	{
		B.ResetShape();
		for (int& n : LevelSamples) n = 0;
		Samples = 0;
	}

	float FBodyShape::ProfileAt(const float* Profile, float t)
	{
		if (t <= Levels[0]) return Profile[0];
		for (int i = 1; i < 6; i++)
			if (t <= Levels[i]) return FMath::Lerp(Profile[i - 1], Profile[i], (t - Levels[i - 1]) / (Levels[i] - Levels[i - 1]));
		return Profile[5];
	}

	bool FBodyShape::Apply(const TArray<float>& A, const TArray<float>& D, FBodyRig& B, float Dt)
	{
		if (A.Num() != 6) return false;
		const float U = B.Unit(), Rate = 1.f - FMath::Exp(-Dt / 0.5f);     // perception already averages; this only eases steps
		float Ra[6], Rd[6];
		int Na = 0;
		for (int k = 0; k < 6; k++)
		{
			float a, d;
			FRestSkeleton::BodyModel(Levels[k], B.HipW, B.ShoulderW, U, a, d);
			Ra[k] = A[k] > 0.f ? FMath::Clamp((A[k] - ClothingAllowance) / a, 0.7f, 1.8f) : 0.f;
			Rd[k] = D.Num() == 6 && D[k] > 0.f ? FMath::Clamp((D[k] - ClothingAllowance) / d, 0.65f, 2.2f) : 0.f;
			Na += Ra[k] > 0.f;
		}
		if (Na == 0) return false;
		// levels not measured (an arm in the way, a bag) take the nearest measured one's ratio
		auto Fill = [](float* R)
		{
			for (int k = 0; k < 6; k++)
			{
				if (R[k] > 0.f) continue;
				for (int o = 1; o < 6 && R[k] <= 0.f; o++)
					for (const int j : { k - o, k + o })
						if (j >= 0 && j < 6 && R[j] > 0.f && R[k] <= 0.f) R[k] = -R[j];
			}
			for (int k = 0; k < 6; k++) R[k] = FMath::Abs(R[k]);
		};
		Fill(Ra); Fill(Rd);
		for (int k = 0; k < 6; k++)
		{
			const float Rdk = Rd[k] > 0.f ? Rd[k] : 1.f + (Ra[k] - 1.f) * 0.6f;
			B.Profile[k] = Samples == 0 ? Ra[k] : Lerp(B.Profile[k], Ra[k], Rate);
			B.DepthProfile[k] = Samples == 0 ? Rdk : Lerp(B.DepthProfile[k], Rdk, Rate);
		}
		Samples++;
		return true;
	}

	static float CatmullRom(float p0, float p1, float p2, float p3, float x)
	{
		return p1 + 0.5f * x * (p2 - p0 + x * (2.f * p0 - 5.f * p1 + 4.f * p2 - p3 + x * (3.f * (p1 - p2) + p3 - p0)));
	}

	float FBodyMould::AcrossAt(const float* V, float fc)
	{
		fc = FMath::Clamp(fc, 0.f, float(Cols - 1));
		const int c = FMath::Min(int(fc), Cols - 2);
		// mirrored about the centre line; continued straight past the last column
		const float Before = V[c == 0 ? 1 : c - 1], After = c + 2 < Cols ? V[c + 2] : 2.f * V[c + 1] - V[c];
		return CatmullRom(Before, V[c], V[c + 1], After, fc - c);
	}

	float FBodyMould::FrontAt(float t, float s) const
	{
		const float fr = FMath::Clamp((t - T0) / (T1 - T0), 0.f, 1.f) * (Rows - 1), fc = FMath::Abs(s) / ColStep;
		const int r = FMath::Min(int(fr), Rows - 2);
		return CatmullRom(AcrossAt(Front[FMath::Max(r - 1, 0)], fc), AcrossAt(Front[r], fc), AcrossAt(Front[r + 1], fc),
			AcrossAt(Front[FMath::Min(r + 2, Rows - 1)], fc), fr - r);
	}

	float FBodyMould::ShoulderAt(float x) const
	{
		if (x <= ShX[0]) return Sh[0];
		for (int i = 1; i < ShN; i++)
			if (x <= ShX[i]) return Lerp(Sh[i - 1], Sh[i], (x - ShX[i - 1]) / (ShX[i] - ShX[i - 1]));
		return Sh[ShN - 1];
	}

	void FBodyShape::ApplyMould(const TArray<float>& GZ, const TArray<float>& Sh, FBodyRig& B, float Dt)
	{
		constexpr int R = FBodyMould::Rows, C = FBodyMould::Cols;
		if (GZ.Num() != R * C) return;
		float G[R][C];
		int Have = 0;
		for (int r = 0; r < R; r++)
		{
			for (int c = 0; c < C; c++) G[r][c] = GZ[r * C + c] > 0.f ? FMath::Max(GZ[r * C + c] - ClothingAllowance, 1e-3f) : GZ[r * C + c];
			Have += G[r][0] > 0.f;
		}
		if (Have < R * 3 / 4) return;                         // not enough of it yet: the ovals do until then
		// a hole in a row: as its nearest measured neighbour in that row, along the oval through it
		auto Oval = [](int c) { const float s = c * FBodyMould::ColStep; return FMath::Sqrt(1.f - s * s); };
		for (int r = 0; r < R; r++)
		{
			if (G[r][0] <= 0.f) continue;
			for (int c = 1; c < C; c++)
			{
				if (G[r][c] > 0.f) continue;
				int n = c - 1;
				for (int o = c + 1; o < C && G[r][c - 1] <= 0.f; o++) if (G[r][o] > 0.f) { n = o; break; }
				G[r][c] = -FMath::Abs(G[r][n]) * Oval(c) / Oval(n);            // (negative: filled, not to be copied from in turn)
			}
			for (int c = 1; c < C; c++) G[r][c] = FMath::Abs(G[r][c]);
		}
		// a row not measured at all: the nearest one that is
		for (int r = 0; r < R; r++)
		{
			if (G[r][0] > 0.f) continue;
			for (int o = 1; o < R && G[r][0] <= 0.f; o++)
				for (const int j : { r - o, r + o })
					if (j >= 0 && j < R && G[j][0] > 0.f && G[r][0] <= 0.f)
						for (int c = 0; c < C; c++) G[r][c] = -G[j][c];
		}
		for (int r = 0; r < R; r++) for (int c = 0; c < C; c++) G[r][c] = FMath::Abs(G[r][c]);

		FBodyMould& M = B.Mould;
		const float Rate = 1.f - FMath::Exp(-Dt / 0.5f);       // perception already averages; this only eases steps
		for (int r = 0; r < R; r++)
			for (int c = 0; c < C; c++)
			{
				// a little smoothing up and down: depth noise shows as rings around the body otherwise
				const float v = 0.25f * G[FMath::Max(r - 1, 0)][c] + 0.5f * G[r][c] + 0.25f * G[FMath::Min(r + 1, R - 1)][c];
				M.Front[r][c] = M.bValid ? Lerp(M.Front[r][c], v, Rate) : v;
			}
		M.bValid = true;
		bool bSh = Sh.Num() == FBodyMould::ShN;
		for (int i = 0; bSh && i < FBodyMould::ShN; i++) bSh = Sh[i] > 0.f;
		if (bSh)
		{
			for (int i = 0; i < FBodyMould::ShN; i++) M.Sh[i] = M.bShoulders ? Lerp(M.Sh[i], Sh[i], Rate) : Sh[i];
			M.bShoulders = true;
		}
	}

	static int SampleMask(const TArray<uint8>& Mask, int MW, int MH, const V2& p)
	{
		const int x = FMath::FloorToInt(p.X), y = FMath::FloorToInt(p.Y);
		return x < 0 || y < 0 || x >= MW || y >= MH ? -1 : Mask[y * MW + x];
	}

	/** Distance (m) from c along dir to where the mask stops being the person, or -1. */
	static float ScanEdge(const TArray<uint8>& Mask, int MW, int MH, const FVector4f M[3], const V3& c, const V3& dir, float MaxLen)
	{
		V2 p0, p1;
		if (!ProjectRows(M, c, p0) || !ProjectRows(M, c + dir * MaxLen, p1)) return -1.f;
		p0.X *= MW; p0.Y *= MH; p1.X *= MW; p1.Y *= MH;
		const int Steps = FMath::CeilToInt(V2::Distance(p0, p1) * 2.f);      // half-pixel steps
		if (Steps < 4 || SampleMask(Mask, MW, MH, p0) < 128) return -1.f;
		for (int i = 1; i <= Steps; i++)
		{
			const int m = SampleMask(Mask, MW, MH, p0 + (p1 - p0) * (i / float(Steps)));
			if (m < 0) return -1.f;
			if (m < 128) return MaxLen * (i - 0.5f) / Steps;
		}
		return -1.f;
	}

	/** Whether this side's arm (or hand) is at this height and within a few cm of the silhouette edge. */
	static bool ArmTouches(const V3* J, bool bRight, const V3& HipC, const V3& Up, const V3& c, const V3& Dir, float H, float Edge, float U)
	{
		const int s = bRight ? LM::RShoulder : LM::LShoulder, e = bRight ? LM::RElbow : LM::LElbow;
		const int w = bRight ? LM::RWrist : LM::LWrist, k = bRight ? LM::RIndex : LM::LIndex;
		auto Near = [&](const V3& a, const V3& b)
		{
			const float ha = Dot(a - HipC, Up) - H, hb = Dot(b - HipC, Up) - H, Margin = 0.06f * U;
			if (FMath::Min(ha, hb) > Margin || FMath::Max(ha, hb) < -Margin) return false;
			const float t = FMath::Abs(ha - hb) > 1e-4f ? Clamp01(ha / (ha - hb)) : 0.5f;
			const float Lat = Dot(LerpU(a, b, t) - c, Dir), Ra = 0.055f * U;
			return Lat + Ra > Edge - 0.015f && Lat - Ra < Edge + 0.03f;
		};
		return Near(J[s], J[e]) || Near(J[e], J[w]) || Near(J[w], J[k]);
	}

	void FBodyShape::Measure(const TArray<uint8>& Mask, int MW, int MH, const FVector4f M[3], const V3* InJ,
		FBodyRig& B, const V3& CamPos, float Dt)
	{
		if (MW <= 0 || MH <= 0 || Mask.Num() < MW * MH) return;
		if (B.Vis[LM::LShoulder] < 0.6f || B.Vis[LM::RShoulder] < 0.6f || B.Vis[LM::LHip] < 0.5f || B.Vis[LM::RHip] < 0.5f) return;
		const V3 HipC = (InJ[LM::LHip] + InJ[LM::RHip]) * 0.5f, ShC = (InJ[LM::LShoulder] + InJ[LM::RShoulder]) * 0.5f;
		const float L = FVector3f::Distance(HipC, ShC);
		if (L < 0.2f) return;
		const V3 Up = (ShC - HipC) / L;
		const V3 ChestR = Normalized(ProjectOnPlane(InJ[LM::RShoulder] - InJ[LM::LShoulder], Up));
		V3 PelvisR = Normalized(ProjectOnPlane(InJ[LM::RHip] - InJ[LM::LHip], Up));
		if (PelvisR.SizeSquared() < 0.5f) PelvisR = ChestR;
		const V3 Fwd = Cross(ChestR, Up);
		if (FMath::Abs(Dot(Fwd, Normalized(CamPos - ShC))) < FMath::Cos(FMath::DegreesToRadians(MaxYawDeg))) return;

		const float U = B.Unit(), Rate = 1.f - FMath::Exp(-Dt / TimeConstant);
		for (int k = 0; k < 6; k++)
		{
			const float t = Levels[k];
			const V3 R = SlerpDir(PelvisR, ChestR, SmoothStep(0.f, 1.f, (t - 0.1f) / 0.7f));
			const V3 c = HipC + Up * (t * L);
			float Sum = 0.f;
			int N = 0;
			for (int Side = -1; Side <= 1; Side += 2)
			{
				const V3 Dir = R * float(Side);
				const float Edge = ScanEdge(Mask, MW, MH, M, c, Dir, 0.45f * U);
				if (Edge <= 0.f || ArmTouches(InJ, Side > 0, HipC, Up, c, Dir, t * L, Edge, U)) continue;
				Sum += Edge;
				N++;
			}
			// at the hips and waist an arm hanging by the side merges with the silhouette more often
			// than not: only believe a width there when both sides are clear
			if (N == 0 || (t < 0.3f && N < 2)) continue;
			float a, d;
			FRestSkeleton::BodyModel(t, B.HipW, B.ShoulderW, U, a, d);
			const float r = Sum / N / a;
			if (r < 0.8f || r > 1.3f) continue;                                             // not a body: clothes, a bag, a bad mask
			if (LevelSamples[k] > 15 && FMath::Abs(r - B.Profile[k]) > 0.2f) continue;   // a bag, a loose sleeve, ...
			B.Profile[k] = LevelSamples[k] == 0 ? r : Lerp(B.Profile[k], r, Rate);
			B.DepthProfile[k] = 1.f + (B.Profile[k] - 1.f) * 0.6f;      // unseen: wider usually means deeper too
			LevelSamples[k]++;
			Samples++;
		}
	}
}
