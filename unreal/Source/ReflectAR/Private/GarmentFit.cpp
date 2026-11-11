#include "GarmentFit.h"
#include "Algo/Sort.h"

namespace ReflectAR
{
	TSharedPtr<const FGarmentAsset> FindGarmentAsset(const FString& Name)
	{
		static TMap<FString, TSharedPtr<const FGarmentAsset>> Cache;
		if (const TSharedPtr<const FGarmentAsset>* Found = Cache.Find(Name)) return *Found;
		TSharedPtr<FGarmentAsset> A = MakeShared<FGarmentAsset>();
		if (!LoadGarmentAsset(Name, *A) || A->Pos.Num() == 0 || A->Tris.Num() == 0) A.Reset();
		else UE_LOG(LogTemp, Display, TEXT("ReflectAR: garment mesh %s: %d vertices, %d triangles"), *Name, A->Pos.Num(), A->Tris.Num() / 3);
		Cache.Add(Name, A);
		return A;
	}

	namespace
	{
		/** The body the garments are authored on (garments/gp_body.py). */
		FRestSkeleton ReferenceSkeleton()
		{
			FRestSkeleton r;
			r.hipC = V3::ZeroVector;
			r.shC = V3(0, r.L, 0);
			r.lSh = r.shC - VRight * (r.sw / 2); r.rSh = r.shC + VRight * (r.sw / 2);
			r.lEl = r.lSh + FRestSkeleton::ArmDir(-1) * r.upperL; r.lWr = r.lEl + FRestSkeleton::ArmDir(-1) * r.foreL;
			r.rEl = r.rSh + FRestSkeleton::ArmDir(+1) * r.upperR; r.rWr = r.rEl + FRestSkeleton::ArmDir(+1) * r.foreR;
			return r;
		}

		float SegDist(const V3& p, const V3& a, const V3& b)
		{
			const V3 ab = b - a;
			const float t = Clamp01(Dot(p - a, ab) / FMath::Max(ab.SizeSquared(), 1e-8f));
			return V3::Distance(p, a + ab * t);
		}

		/** Skin weights as GarmentMesher gives its own meshes, except that the sleeves (Arm, 0..1)
		 * are known here rather than guessed from the position. */
		FBoneWeights Weights(const V3& p, const FRestSkeleton& s, float HemY, float Arm)
		{
			const V3 Mid(0, s.L * 0.45f, 0);
			float Dist[6], W[6];
			Dist[0] = SegDist(p, V3(0, HemY - 0.05f, 0), Mid);
			Dist[1] = FMath::Min(SegDist(p, Mid, s.shC), SegDist(p, s.lSh, s.rSh));
			Dist[2] = SegDist(p, s.lSh, s.lEl); Dist[3] = SegDist(p, s.rSh, s.rEl);
			Dist[4] = SegDist(p, s.lEl, s.lWr); Dist[5] = SegDist(p, s.rEl, s.rWr);
			if (p.X < 0) { Dist[3] = Dist[5] = 1.f; } else { Dist[2] = Dist[4] = 1.f; }
			float SumBody = 0, SumArm = 0;
			for (int b = 0; b < 6; b++) { const float dd = FMath::Max(Dist[b], 0.01f); W[b] = 1.f / (dd * dd * dd * dd); (b < 2 ? SumBody : SumArm) += W[b]; }
			for (int b = 0; b < 6; b++) W[b] *= b < 2 ? (1.f - Arm) / SumBody : Arm / FMath::Max(SumArm, 1e-9f);
			int Idx[6] = { 0, 1, 2, 3, 4, 5 };
			Algo::Sort(Idx, [&W](int x, int y) { return W[x] > W[y]; });
			const float S4 = W[Idx[0]] + W[Idx[1]] + W[Idx[2]] + W[Idx[3]];
			FBoneWeights Bw;
			for (int k = 0; k < 4; k++) { Bw.Bone[k] = Idx[k]; Bw.W[k] = W[Idx[k]] / S4; }
			return Bw;
		}

		/**
		 * How much of each vertex belongs to a sleeve (0 torso .. 1 sleeve), blended over a few
		 * centimetres across the armhole. The pattern pieces are the UV islands: an island lying
		 * out along an arm is a sleeve or a cuff. A mesh in one piece falls back to its position.
		 */
		void SleeveWeights(const FGarmentAsset& A, const FRestSkeleton& Ref, TArray<float>& Arm)
		{
			const int N = A.Pos.Num(), Rn = A.RenderToSim.Num();
			TArray<int32> Parent;
			Parent.SetNumUninitialized(Rn);
			for (int i = 0; i < Rn; i++) Parent[i] = i;
			auto Find = [&Parent](int i) { while (Parent[i] != i) { Parent[i] = Parent[Parent[i]]; i = Parent[i]; } return i; };
			for (int t = 0; t + 2 < A.RenderTris.Num(); t += 3)
				for (int e = 1; e < 3; e++)
				{
					const int a = Find(A.RenderTris[t]), b = Find(A.RenderTris[t + e]);
					if (a != b) Parent[a] = b;
				}
			TMap<int32, V2> Sum;                        // island -> (sum |x|, count)
			for (int i = 0; i < Rn; i++) { V2& s = Sum.FindOrAdd(Find(i)); s.X += FMath::Abs(A.Pos[A.RenderToSim[i]].X); s.Y += 1.f; }
			const float ArmX = Ref.sw * 0.5f + 0.07f;   // a bodice's mean |x| is ~0.1, a sleeve's 0.3 or more
			TArray<float> Hit, Cnt;
			Hit.SetNumZeroed(N); Cnt.SetNumZeroed(N);
			bool bAny = false;
			for (int i = 0; i < Rn; i++)
			{
				const V2 s = Sum[Find(i)];
				const bool bArm = Sum.Num() > 1 && s.X / s.Y > ArmX;
				bAny |= bArm;
				Hit[A.RenderToSim[i]] += bArm ? 1.f : 0.f; Cnt[A.RenderToSim[i]] += 1.f;
			}
			Arm.SetNumUninitialized(N);
			for (int i = 0; i < N; i++)
			{
				if (bAny) { Arm[i] = Cnt[i] > 0 ? Hit[i] / Cnt[i] : 0.f; continue; }
				const V3 p = A.Pos[i];
				const float Side = p.X < 0 ? -1.f : 1.f;
				const V3 Rel = p - (Side < 0 ? Ref.lSh : Ref.rSh), Dir = FRestSkeleton::ArmDir(Side);
				const float Along = Dot(Rel, Dir), Radial = (Rel - Dir * Along).Size();
				Arm[i] = Ramp(0.f, 0.08f, Along) * (1.f - Ramp(0.085f, 0.12f, Radial));
			}
			// blend across the armhole seam: a few rounds of averaging with the neighbours (~4 cm)
			TArray<int32> EA, EB;
			for (int t = 0; t + 2 < A.Tris.Num(); t += 3)
				for (int e = 0; e < 3; e++) { EA.Add(A.Tris[t + e]); EB.Add(A.Tris[t + (e + 1) % 3]); }
			TArray<float> Acc, Num;
			for (int It = 0; It < 12; It++)
			{
				Acc.Init(0.f, N); Num.Init(0.f, N);
				for (int e = 0; e < EA.Num(); e++) { Acc[EA[e]] += Arm[EB[e]]; Num[EA[e]] += 1.f; Acc[EB[e]] += Arm[EA[e]]; Num[EB[e]] += 1.f; }
				for (int i = 0; i < N; i++) if (Num[i] > 0) Arm[i] = 0.5f * Arm[i] + 0.5f * Acc[i] / Num[i];
			}
		}

		/** Ribbed bands: the triangles of the rib region (vertex colour B), clipped at its edge. */
		void BuildRibs(const FGarmentAsset& A, FGarmentMeshData& D)
		{
			const TArray<float>& R = A.Rib;
			TMap<uint64, int32> Map;
			auto Vert = [&](int ra, int rb) -> int32               // render vertices: on the edge ra-rb where the band ends
			{
				if (ra > rb) Swap(ra, rb);
				const uint64 Key = (uint64(uint32(ra)) << 32) | uint32(rb);
				if (const int32* Found = Map.Find(Key)) return *Found;
				const int a = A.RenderToSim[ra], b = A.RenderToSim[rb];
				const float fa = R[a] - 0.5f, fb = R[b] - 0.5f;
				const float t = ra == rb || FMath::Abs(fa - fb) < 1e-6f ? 0.f : Clamp01(fa / (fa - fb));
				D.RibVerts.Add({ a, b, t });
				D.RibUV.Add(A.UV[ra] + (A.UV[rb] - A.UV[ra]) * t);
				return Map.Add(Key, D.RibVerts.Num() - 1);
			};
			for (int t = 0; t + 2 < A.RenderTris.Num(); t += 3)
			{
				const int v[3] = { A.RenderTris[t], A.RenderTris[t + 1], A.RenderTris[t + 2] };
				bool In[3]; int nIn = 0;
				for (int e = 0; e < 3; e++) { In[e] = R[A.RenderToSim[v[e]]] > 0.5f; nIn += In[e]; }
				if (nIn == 0) continue;
				int32 Poly[4]; int n = 0;
				for (int e = 0; e < 3; e++)
				{
					const int a = v[e], b = v[(e + 1) % 3];
					if (In[e]) Poly[n++] = Vert(a, a);
					if (In[e] != In[(e + 1) % 3]) Poly[n++] = Vert(a, b);
				}
				for (int k = 1; k + 1 < n; k++) D.RibTris.Append({ Poly[0], Poly[k], Poly[k + 1] });
			}
		}
	}

	TSharedPtr<FGarmentMeshData> FitGarmentAsset(const FGarmentAsset& A, const FRestSkeleton& S)
	{
		TSharedPtr<FGarmentMeshData> Out = MakeShared<FGarmentMeshData>();
		FGarmentMeshData& D = *Out;
		const FRestSkeleton Ref = ReferenceSkeleton();
		const int N = A.Pos.Num();
		TArray<float> Arm;
		SleeveWeights(A, Ref, Arm);

		// ---- reference body -> the user's
		const float u = S.u, kY = S.L / Ref.L, kSw = S.sw / Ref.sw;
		float aTop0, dTop0;
		Ref.TorsoRadii(0.72f * Ref.L, aTop0, dTop0);
		// Fabric hangs from the most forward part of the body above it: off a belly or a chest it
		// falls nearly straight down and doesn't tuck back in underneath. So free fabric is fitted
		// to the body's front as cloth hung over it would lie (each column of the mould, from the
		// shoulders down, never retreating by more than a slight lean), and only what holds on to
		// the body, a ribbed hem, to the body itself.
		FRestSkeleton Hung = S;
		for (int r = FBodyMould::Rows - 2; S.Mould.bValid && r >= 0; r--)
			for (int c = 0; c < FBodyMould::Cols; c++)
				Hung.Mould.Front[r][c] = FMath::Max(S.Mould.Front[r][c], Hung.Mould.Front[r + 1][c] - 0.12f * S.L * (FBodyMould::T1 - FBodyMould::T0) / (FBodyMould::Rows - 1));
		// arms carry more on a heavier build, though less than the torso does
		float Girth = 0.f;
		for (int i = 2; i < 6; i++) Girth += 0.25f * FMath::Sqrt(FMath::Max(S.Profile[i] * S.DepthProfile[i], 0.3f));
		const float kArm = u * FMath::Clamp(FMath::Lerp(1.f, Girth, 0.5f), 0.95f, 1.3f);

		D.Rest.SetNumUninitialized(N);
		for (int i = 0; i < N; i++)
		{
			const V3 p = A.Pos[i];
			// torso: the section at this height, reshaped; the distance off the body kept
			const float t = p.Y / Ref.L;
			float a0, d0, a1, c1, dF, dB, aTop1, cTop, dTopF, dTopB;
			Ref.TorsoRadii(FMath::Min(t, 1.f) * Ref.L, a0, d0);
			const float q = FMath::Sqrt(FMath::Square(p.X / a0) + FMath::Square(p.Z / d0));
			// the user's section there: the same place around the body, on their measured surface
			// (the front of it from the mould of their torso, when there is one)
			const float s = q > 1e-4f ? FMath::Min(FMath::Abs(p.X) / (a0 * q), 1.f) : 0.f;
			Hung.TorsoSection(FMath::Min(t, 1.f) * S.L, s, a1, c1, dF, dB);
			if (const float Grip = Clamp01(A.Elastic[i]); Grip > 0.f && S.Mould.bValid)
			{
				float aB, cB, dFB, dBB;
				S.TorsoSection(FMath::Min(t, 1.f) * S.L, s, aB, cB, dFB, dBB);
				c1 = FMath::Lerp(c1, cB, Grip); dF = FMath::Lerp(dF, dFB, Grip); dB = FMath::Lerp(dB, dBB, Grip);
			}
			Hung.TorsoSection(0.72f * S.L, s, aTop1, cTop, dTopF, dTopB);
			const float d1 = p.Z >= 0.f ? dF : dB, dTop1 = p.Z >= 0.f ? dTopF : dTopB;
			// toward the shoulders the width is the skeleton's (the shoulder joints), not the build's,
			// and the yoke sits on the shoulders, wherever the chest or belly below it stands
			const float kA = FMath::Lerp(a1 / a0, FMath::Lerp(aTop1 / aTop0, kSw, 0.6f), Ramp(0.72f, 0.95f, t));
			const float kD = FMath::Lerp(d1 / d0, FMath::Lerp(dTop1 / dTop0, u, 0.7f), Ramp(0.8f, 1.05f, t));
			const float c = c1 * (1.f - Ramp(0.8f, 1.02f, t));
			V3 Body(p.X * kA, 0.f, c + p.Z * kD);                 // plain scaling: near the axis, where "off the body" means nothing
			if (q > 0.35f)
			{
				const V2 e(p.X / q, p.Z / q);                     // where the ray from the axis leaves the reference body
				const V3 Off(e.X * kA + (p.X - e.X) * u, 0.f, c + e.Y * kD + (p.Z - e.Y) * u);
				Body = LerpV(Body, Off, Ramp(0.35f, 0.8f, q));
			}
			Body.Y = t <= 1.f ? p.Y * kY : S.L + (p.Y - Ref.L) * u;

			// sleeve: along the user's arm
			const float Side = p.X < 0 ? -1.f : 1.f;
			const V3 Dir = FRestSkeleton::ArmDir(Side), Rel = p - (Side < 0 ? Ref.lSh : Ref.rSh);
			const float Along = Dot(Rel, Dir);
			const float Up0 = Side < 0 ? Ref.upperL : Ref.upperR, Up1 = Side < 0 ? S.upperL : S.upperR;
			const float Fo0 = Side < 0 ? Ref.foreL : Ref.foreR, Fo1 = Side < 0 ? S.foreL : S.foreR;
			const float Along1 = Along <= 0.f ? Along * u : Along <= Up0 ? Along * Up1 / Up0 : Up1 + (Along - Up0) * Fo1 / Fo0;
			const V3 Sleeve = (Side < 0 ? S.lSh : S.rSh) + Dir * Along1 + (Rel - Dir * Along) * kArm;

			D.Rest[i] = LerpV(Body, Sleeve, Arm[i]);

			// The reference body's shoulder line is a level bar, so the garments come with square
			// shoulders. A person's slope down from the neck: the yoke is raised at the neck and
			// lowered a little at the shoulder points, fading out down the chest and up a tall
			// collar. By the user's own shoulders where the depth camera measured their tops (the
			// authored yoke lies 5.5 cm above the joints), else by about 16 degrees.
			const float ShX = FMath::Abs(p.X) / (Ref.sw * 0.5f);
			float Slope = u * FMath::Lerp(0.028f, -0.010f, Clamp01((FMath::Abs(p.X) - 0.05f) / (Ref.sw * 0.5f - 0.05f)));
			if (S.Mould.bShoulders) Slope = FMath::Clamp(S.Mould.ShoulderAt(ShX) - 0.055f * u, -0.012f * u, 0.06f * u);
			D.Rest[i].Y += Slope * Ramp(0.62f, 1.f, t) * (1.f - Ramp(Ref.L + 0.08f, Ref.L + 0.16f, p.Y));
		}

		D.Tris = A.Tris;
		D.RenderToSim = A.RenderToSim; D.UV = A.UV; D.RenderTris = A.RenderTris;
		GarmentMesher::Normals(D.Rest, D.Tris, D.RestN);
		// outward: away from the torso's axis / the arm's, on balance
		double Out2 = 0;
		for (int i = 0; i < N; i++) if (Arm[i] < 0.2f && D.Rest[i].Y < 0.7f * S.L) Out2 += D.RestN[i].X * D.Rest[i].X + D.RestN[i].Z * D.Rest[i].Z;
		if (Out2 < 0)
		{
			for (int t = 0; t + 2 < D.Tris.Num(); t += 3) { Swap(D.Tris[t + 1], D.Tris[t + 2]); Swap(D.RenderTris[t + 1], D.RenderTris[t + 2]); }
			for (V3& n : D.RestN) n = -n;
		}

		float HemY = 1e9f;
		for (int i = 0; i < N; i++) if (Arm[i] < 0.5f) HemY = FMath::Min(HemY, D.Rest[i].Y);
		D.Weights.SetNumUninitialized(N); D.Looseness.SetNumUninitialized(N); D.Band.SetNumUninitialized(N);
		for (int i = 0; i < N; i++)
		{
			D.Weights[i] = Weights(D.Rest[i], S, HemY, Arm[i]);
			D.Looseness[i] = 1.f - Clamp01(A.Pin[i]);
			D.Band[i] = Clamp01(A.Elastic[i]);
		}
		BuildRibs(A, D);
		return Out;
	}
}
