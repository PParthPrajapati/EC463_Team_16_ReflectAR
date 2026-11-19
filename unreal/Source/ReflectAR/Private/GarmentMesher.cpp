#include "GarmentMesher.h"
#include "Algo/Sort.h"

namespace ReflectAR
{
	// ================================================================ rest skeleton
	FRestSkeleton FRestSkeleton::From(const FBodyRig& B)
	{
		FRestSkeleton r;
		r.L = B.TorsoLen; r.sw = B.ShoulderW; r.hipW = B.HipW; r.u = B.Unit();
		r.upperL = B.UpperL; r.upperR = B.UpperR; r.foreL = B.ForeL; r.foreR = B.ForeR;
		for (int i = 0; i < 6; i++) { r.Profile[i] = B.Profile[i]; r.DepthProfile[i] = B.DepthProfile[i]; }
		r.Mould = B.Mould;
		r.hipC = V3::ZeroVector;
		r.shC = V3(0, r.L, 0);
		r.lSh = r.shC - VRight * (r.sw / 2);
		r.rSh = r.shC + VRight * (r.sw / 2);
		r.lEl = r.lSh + ArmDir(-1) * r.upperL; r.lWr = r.lEl + ArmDir(-1) * r.foreL;
		r.rEl = r.rSh + ArmDir(+1) * r.upperR; r.rWr = r.rEl + ArmDir(+1) * r.foreR;
		return r;
	}

	V3 FRestSkeleton::ArmDir(float Side)
	{
		const float a = FMath::DegreesToRadians(ArmDropDeg);
		return V3(Side * FMath::Cos(a), -FMath::Sin(a), 0.f);
	}

	// piecewise linear through knots t = -0.3, 0, 0.3, 0.7, 1
	static float Pw(float t, float v0, float v1, float v2, float v3, float v4)
	{
		if (t <= -0.3f) return v0;
		if (t <= 0.f) return Lerp(v0, v1, (t + 0.3f) / 0.3f);
		if (t <= 0.3f) return Lerp(v1, v2, t / 0.3f);
		if (t <= 0.7f) return Lerp(v2, v3, (t - 0.3f) / 0.4f);
		if (t <= 1.f) return Lerp(v3, v4, (t - 0.7f) / 0.3f);
		return v4;
	}

	void FRestSkeleton::BodyModel(float t, float HipW, float SW, float U, float& a, float& d)
	{
		const float hipA = HipW * 0.68f + 0.02f * U, chestA = SW * 0.5f * 0.9f;
		const float waistA = Lerp(hipA, chestA, 0.45f) * 0.93f;
		a = Pw(t, hipA * 1.04f, hipA, waistA, chestA, chestA * 0.92f);
		d = Pw(t, 0.11f, 0.105f, 0.098f, 0.108f, 0.075f) * U;
	}

	void FRestSkeleton::TorsoRadii(float y, float& a, float& d) const
	{
		float c, dF;
		TorsoSection(y, 0.f, a, c, dF, d);                  // with a mould: half the body's thickness
	}

	void FRestSkeleton::TorsoSection(float y, float s, float& a, float& c, float& dF, float& dB) const
	{
		const float t = y / L;
		float d;
		BodyModel(t, hipW, sw, u, a, d);
		const float Pa = FBodyShape::ProfileAt(Profile, t);
		a *= Pa;
		if (!Mould.bValid)
		{
			c = 0.f;
			dF = dB = d * FBodyShape::ProfileAt(DepthProfile, t);
			return;
		}
		const float Back = d * (1.f + (Pa - 1.f) * 0.6f);      // unseen: a wider body is usually deeper behind too
		const float F0 = FMath::Clamp(Mould.FrontAt(t, 0.f), 0.5f * d, 2.8f * d);
		c = (F0 - Back) * 0.5f; dB = (F0 + Back) * 0.5f;
		// measured out to 0.8 of the half-width; the oval through that closes the side
		const float sc = FMath::Min(FMath::Abs(s), (FBodyMould::Cols - 1) * FBodyMould::ColStep);
		dF = FMath::Clamp((Mould.FrontAt(t, sc) - c) / FMath::Sqrt(1.f - sc * sc), 0.6f * dB, 1.8f * dB);
	}

	namespace GarmentMesher
	{
		// ------------------------------------------------------------ SDF helpers
		static float SMin(float a, float b, float k)
		{
			const float h = Clamp01(0.5f + 0.5f * (b - a) / k);
			return FMath::Lerp(b, a, h) - k * h * (1.f - h);
		}
		static float SMax(float a, float b, float k) { return -SMin(-a, -b, k); }

		static float Capsule(const V3& p, const V3& a, const V3& b, float ra, float rb)
		{
			const V3 ab = b - a;
			const float t = Clamp01(Dot(p - a, ab) / FMath::Max(ab.SizeSquared(), 1e-8f));
			return V3::Distance(p, a + ab * t) - FMath::Lerp(ra, rb, t);
		}

		struct FCtx
		{
			FRestSkeleton s;
			const FGarmentSpec* g = nullptr;
			float Cell = DefaultCell;
			float hemY = 0, neckR = 0, e = 0;
			V3 lEnd, rEnd, lDir, rDir;
			bool lLong = false, rLong = false;
			// garment half-width / half-depth (ease included) vs height
			TArray<float> ga, gd;
			float y0 = 0, dy = 1;

			void Radii(float y, float& a, float& d) const
			{
				float f = FMath::Clamp((y - y0) / dy, 0.f, ga.Num() - 1.001f);
				const int i = int(f); f -= i;
				a = FMath::Lerp(ga[i], ga[i + 1], f);
				d = FMath::Lerp(gd[i], gd[i + 1], f);
			}
		};

		static V3 SleeveEnd(const FRestSkeleton& s, float Side, float Frac, V3& Dir, bool& bReachesForearm)
		{
			const float up = Side < 0 ? s.upperL : s.upperR, fo = Side < 0 ? s.foreL : s.foreR;
			const V3 sh = Side < 0 ? s.lSh : s.rSh;
			const float len = Frac * (up + fo);
			Dir = FRestSkeleton::ArmDir(Side);
			bReachesForearm = len > up;
			return sh + Dir * len;
		}

		/** Garment cross-section vs height: the body plus ease, hanging straight down from the
		 * widest point above (by fit) below the chest, pulled in at a hem band. */
		static void BuildTables(FCtx& c)
		{
			constexpr int N = 128;
			const FRestSkeleton& s = c.s; const FGarmentSpec& g = *c.g;
			c.y0 = -0.4f * s.L; c.dy = (s.L - c.y0) / (N - 1);
			c.ga.SetNumZeroed(N); c.gd.SetNumZeroed(N);
			TArray<float> ba, bd;
			ba.SetNumZeroed(N); bd.SetNumZeroed(N);
			for (int i = 0; i < N; i++) s.TorsoRadii(c.y0 + i * c.dy, ba[i], bd[i]);
			const float HangTop = 0.72f * s.L, Band = 0.06f * s.u, Fit = Clamp01(g.Fit);
			float MaxA = 0, MaxD = 0;
			for (int i = N - 1; i >= 0; i--)
			{
				const float y = c.y0 + i * c.dy;
				if (y >= HangTop) { MaxA = ba[i]; MaxD = bd[i]; }
				else { MaxA = FMath::Max(MaxA, ba[i]); MaxD = FMath::Max(MaxD, bd[i]); }
				float a = Lerp(ba[i], MaxA, Fit) + c.e, d = Lerp(bd[i], MaxD, Fit) + c.e;
				if (g.HemBand > 0.f)
				{
					const float k = (1.f - SmoothStep(0.f, 1.f, (y - c.hemY) / Band)) * Clamp01(g.HemBand);
					a = Lerp(a, ba[i] + 0.35f * c.e, k);
					d = Lerp(d, bd[i] + 0.35f * c.e, k);
				}
				c.ga[i] = a; c.gd[i] = d;
			}
		}

		static float Sdf(const FCtx& c, const V3& p)
		{
			const FRestSkeleton& s = c.s; const FGarmentSpec& g = *c.g;
			const float u = s.u, e = c.e;
			// torso: elliptic cylinder with the garment's cross-section, rounded into the shoulders
			float a, d;
			c.Radii(p.Y, a, d);
			float Torso = (FMath::Sqrt(p.X * p.X / (a * a) + p.Z * p.Z / (d * d)) - 1.f) * FMath::Min(a, d);
			Torso = SMax(Torso, p.Y - (s.L - 0.02f * u), 0.05f * u);
			// the garment rests on the shoulders: the ease is around the body, not on top of it
			const float ShR = 0.054f * u + FMath::Min(e, 0.006f * u);
			const float Shoulders = Capsule(p, s.lSh + VUp * 0.005f, s.rSh + VUp * 0.005f, ShR, ShR);
			float Body = SMin(Torso, Shoulders, 0.07f * u);
			// neck band rising slightly above the shoulders (collar base)
			const bool bCollar = g.Neck == ENeckline::Collar;
			const float BandTop = bCollar ? 0.075f * u : 0.025f * u;
			const float Flare = bCollar ? Clamp01((p.Y - s.L) / (0.06f * u)) * 0.028f * u : 0.f;   // collar rolls outward
			const float NeckBand = FMath::Max(V2(p.X, p.Z + 0.012f * u).Size() - (c.neckR + 0.012f * u + Flare),
				FMath::Abs(p.Y - (s.L + BandTop * 0.5f)) - (BandTop * 0.5f + 0.012f * u));
			if (bCollar || g.Neck == ENeckline::HoodDown) Body = SMin(Body, NeckBand, 0.03f * u);

			if (g.bTank)
			{
				Body = SMax(Body, p.Y - (s.L - 0.075f * u), 0.02f * u);
				for (int Side = -1; Side <= 1; Side += 2)
				{
					const float x = Side * s.sw * 0.27f;
					const V3 f(x, s.L - 0.1f * u, 0.09f * u + e), Top(x, s.L + 0.045f * u, -0.005f), b(x, s.L - 0.1f * u, -0.08f * u - e);
					const float Strap = FMath::Min(Capsule(p, f, Top, 0.018f * u, 0.018f * u), Capsule(p, Top, b, 0.018f * u, 0.018f * u));
					Body = SMin(Body, Strap, 0.02f * u);
				}
			}
			else if (g.Sleeve > 0.f)
			{
				const float se = g.SleeveEase * u;
				for (int Side = -1; Side <= 1; Side += 2)
				{
					const V3 sh = Side < 0 ? s.lSh : s.rSh, End = Side < 0 ? c.lEnd : c.rEnd, Dir = Side < 0 ? c.lDir : c.rDir;
					// the sleeve lies on top of the arm; its ease hangs underneath
					const V3 Hang = Normalized(VUp - Dir * Dot(VUp, Dir)) * (-0.8f * se);
					const V3 Start = sh - VRight * float(Side) * 0.03f * u + VUp * 0.01f * u + Hang;
					const float r0 = 0.056f * u + se, r1 = Lerp(r0, 0.034f * u + 0.6f * se, g.Sleeve) + g.CuffFlare * u;
					float Arm;
					if (g.bCuffBand && (Side < 0 ? c.lLong : c.rLong))
					{
						// gathered cuff at the wrist; the sleeve above blouses over it
						const V3 BandStart = End - Dir * (0.055f * u);
						const float rb = 0.041f * u + 0.25f * se;
						Arm = SMin(Capsule(p, Start, BandStart + Hang * 0.5f, r0, r1), Capsule(p, BandStart, End, rb, rb), 0.012f * u);
					}
					else Arm = Capsule(p, Start, End + Hang, r0, r1);
					Arm = FMath::Max(Arm, Dot(p - End, Dir));                     // flat cuff
					Body = SMin(Body, Arm, 0.06f * u);
				}
			}

			if (g.Neck == ENeckline::HoodDown)                                    // rolled hood lying on the upper back
			{
				V3 q = p - V3(0, s.L + 0.02f * u, -0.075f * u);
				const float ca = FMath::Cos(FMath::DegreesToRadians(-35.f)), sa = FMath::Sin(FMath::DegreesToRadians(-35.f));
				q = V3(q.X, ca * q.Y - sa * q.Z, sa * q.Y + ca * q.Z);
				float Ring = V2(V2(q.X, q.Z).Size() - 0.105f * u, q.Y).Size() - 0.045f * u;
				Ring = FMath::Max(Ring, p.Z - 0.01f * u);                         // back half only
				Body = SMin(Body, Ring, 0.03f * u);
			}
			if (g.Quilt > 0.f)                                                    // puffer baffles
				Body -= g.Quilt * u * 0.5f * (1.f + FMath::Cos(p.Y * PI * 2.f / g.Tile));

			return FMath::Max(Body, c.hemY - p.Y);                                // hem cut
		}

		static V3 Grad(const FCtx& c, const V3& p)
		{
			const float h = c.Cell * 0.35f;
			return V3(Sdf(c, p + VRight * h) - Sdf(c, p - VRight * h),
				Sdf(c, p + VUp * h) - Sdf(c, p - VUp * h),
				Sdf(c, p + VFwd * h) - Sdf(c, p - VFwd * h)) / (2.f * h);
		}

		static int64 EdgeKey(int32 a, int32 b) { return a < b ? (int64(a) << 32) | uint32(b) : (int64(b) << 32) | uint32(a); }

		static TArray<int32> KeepLargestComponent(const TArray<int32>& Tris, int Nv)
		{
			TArray<int32> Parent;
			Parent.SetNumUninitialized(Nv);
			for (int i = 0; i < Nv; i++) Parent[i] = i;
			auto Find = [&Parent](int x) { while (Parent[x] != x) { Parent[x] = Parent[Parent[x]]; x = Parent[x]; } return x; };
			for (int t = 0; t < Tris.Num(); t += 3)
			{
				const int a = Find(Tris[t]), b = Find(Tris[t + 1]);
				Parent[b] = a;
				Parent[Find(Tris[t + 2])] = a;
			}
			TMap<int32, int32> Count;
			for (int t = 0; t < Tris.Num(); t += 3) Count.FindOrAdd(Find(Tris[t]))++;
			int Best = -1, BestN = 0;
			for (const auto& kv : Count) if (kv.Value > BestN) { Best = kv.Key; BestN = kv.Value; }
			TArray<int32> Out;
			Out.Reserve(Tris.Num());
			for (int t = 0; t < Tris.Num(); t += 3)
				if (Find(Tris[t]) == Best) { Out.Add(Tris[t]); Out.Add(Tris[t + 1]); Out.Add(Tris[t + 2]); }
			return Out;
		}

		static void Compact(const TArray<V3>& Verts, const TArray<int32>& Tris, FGarmentMeshData& D)
		{
			TArray<int32> Map;
			Map.Init(-1, Verts.Num());
			D.Rest.Reset(); D.Tris.SetNumUninitialized(Tris.Num());
			for (int t = 0; t < Tris.Num(); t++)
			{
				const int v = Tris[t];
				if (Map[v] < 0) { Map[v] = D.Rest.Num(); D.Rest.Add(Verts[v]); }
				D.Tris[t] = Map[v];
			}
		}

		/** Relax the jagged voxel edges of every opening along the boundary loop. */
		static void SmoothBoundaries(FGarmentMeshData& D)
		{
			TMap<int64, int32> EdgeCount;
			for (int t = 0; t < D.Tris.Num(); t += 3)
				for (int e = 0; e < 3; e++) EdgeCount.FindOrAdd(EdgeKey(D.Tris[t + e], D.Tris[t + (e + 1) % 3]))++;
			TMap<int32, TArray<int32>> Nbr;
			for (const auto& kv : EdgeCount)
			{
				if (kv.Value != 1) continue;
				const int a = int(kv.Key >> 32), b = int(kv.Key & 0xffffffff);
				Nbr.FindOrAdd(a).Add(b);
				Nbr.FindOrAdd(b).Add(a);
			}
			for (int It = 0; It < 6; It++)
			{
				TArray<TPair<int32, V3>> Moved;
				for (const auto& kv : Nbr)
				{
					if (kv.Value.Num() != 2) continue;
					const V3 Avg = (D.Rest[kv.Value[0]] + D.Rest[kv.Value[1]]) * 0.5f;
					Moved.Emplace(kv.Key, LerpU(D.Rest[kv.Key], Avg, 0.5f));
				}
				for (const auto& m : Moved) D.Rest[m.Key] = m.Value;
			}
		}

		void Normals(const TArray<V3>& V, const TArray<int32>& Tris, TArray<V3>& N)
		{
			N.Init(V3::ZeroVector, V.Num());
			for (int t = 0; t < Tris.Num(); t += 3)
			{
				const int a = Tris[t], b = Tris[t + 1], c = Tris[t + 2];
				const V3 Fn = Cross(V[b] - V[a], V[c] - V[a]);
				N[a] += Fn; N[b] += Fn; N[c] += Fn;
			}
			for (V3& n : N) n = Normalized(n);
		}

		static float SegDist(const V3& p, const V3& a, const V3& b, float& t)
		{
			const V3 ab = b - a;
			t = Clamp01(Dot(p - a, ab) / FMath::Max(ab.SizeSquared(), 1e-8f));
			return V3::Distance(p, a + ab * t);
		}

		/** Smooth inverse-distance skin weights to 6 bones, and how loose each vertex may hang. */
		static void Skin(FGarmentMeshData& D, const FRestSkeleton& s, const FGarmentSpec& g, const FCtx& c)
		{
			const int N = D.Rest.Num();
			D.Weights.SetNum(N);
			D.Looseness.SetNum(N);
			D.Band.SetNum(N);
			const V3 Mid(0, s.L * 0.45f, 0);
			float Dist[6], W[6];
			for (int i = 0; i < N; i++)
			{
				const V3 p = D.Rest[i];
				float t0, tl, tr, tlf, trf;
				Dist[0] = SegDist(p, V3(0, c.hemY - 0.05f, 0), Mid, t0);
				Dist[1] = FMath::Min(SegDist(p, Mid, s.shC, t0), SegDist(p, s.lSh, s.rSh, t0));
				Dist[2] = SegDist(p, s.lSh, s.lEl, tl);
				Dist[3] = SegDist(p, s.rSh, s.rEl, tr);
				Dist[4] = SegDist(p, s.lEl, s.lWr, tlf);
				Dist[5] = SegDist(p, s.rEl, s.rWr, trf);
				// the arm bones only own vertices outside the shoulder joint
				const float Side = p.X < 0 ? -1.f : 1.f, OutX = FMath::Abs(p.X) - s.sw * 0.5f;
				if (OutX < 0.f) { Dist[2] = Dist[3] = Dist[4] = Dist[5] = 1.f; }
				else if (Side < 0) { Dist[3] = Dist[5] = 1.f; }
				else { Dist[2] = Dist[4] = 1.f; }

				float Sum = 0;
				for (int b = 0; b < 6; b++) { const float dd = FMath::Max(Dist[b], 0.01f); W[b] = 1.f / (dd * dd * dd * dd); Sum += W[b]; }
				int Idx[6] = { 0, 1, 2, 3, 4, 5 };
				Algo::Sort(Idx, [&W](int x, int y) { return W[x] > W[y]; });
				const float S4 = W[Idx[0]] + W[Idx[1]] + W[Idx[2]] + W[Idx[3]];
				FBoneWeights& Bw = D.Weights[i];
				for (int k = 0; k < 4; k++) { Bw.Bone[k] = Idx[k]; Bw.W[k] = W[Idx[k]] / S4; }

				// looseness: torso -> grows from the shoulder line to the hem; sleeves -> from shoulder to cuff
				const float ArmW = (W[2] + W[3] + W[4] + W[5]) / Sum;
				const float TTorso = Clamp01((s.L * 0.9f - p.Y) / FMath::Max(s.L * 0.9f - c.hemY, 0.05f));
				const float SleeveLen = g.Sleeve * (Side < 0 ? s.upperL + s.foreL : s.upperR + s.foreR);
				const float Along = Side < 0 ? (Dist[2] < Dist[4] ? tl * s.upperL : s.upperL + tlf * s.foreL)
					: (Dist[3] < Dist[5] ? tr * s.upperR : s.upperR + trf * s.foreR);
				const float TArm = SleeveLen > 0.01f ? Clamp01(Along / SleeveLen) : 0.f;
				float Loose = Lerp(TTorso, TArm, ArmW);
				// elastic bands hold on to the body
				float Band = 0.f;
				if (g.HemBand > 0.f) Band = Clamp01(g.HemBand) * (1.f - ArmW) * (1.f - SmoothStep(0.f, 1.f, (p.Y - c.hemY) / (0.07f * s.u)));
				if (g.bCuffBand && ArmW > 0.5f) Band = FMath::Max(Band, 1.f - SmoothStep(0.f, 1.f, (SleeveLen - Along) / (0.07f * s.u)));
				D.Band[i] = Band;
				if (p.Y > s.L - 0.02f * s.u && FMath::Abs(p.X) < s.sw * 0.5f) Loose = 0.f;   // collar / yoke
				D.Looseness[i] = Loose;
			}
		}

		// ------------------------------------------------------------ UV layout
		enum EChart { ChFront = 0, ChBack = 1, ChYoke = 2, ChSleeveL = 3, ChSleeveR = 4 };

		static float ArmWeight(const FBoneWeights& b)
		{
			float w = 0;
			for (int k = 0; k < 4; k++) if (b.Bone[k] >= 2) w += b.W[k];
			return w;
		}

		/** Signed arc length of the ellipse (x, z) = (a sin t, d cos t) from t = 0 to th (Simpson). */
		static float EllipseArc(float a, float d, float th)
		{
			constexpr int n = 8;
			const float h = th / n;
			float Sum = 0;
			for (int i = 0; i <= n; i++)
			{
				const float t = i * h, ct = FMath::Cos(t), st = FMath::Sin(t);
				Sum += FMath::Sqrt(a * a * ct * ct + d * d * st * st) * (i == 0 || i == n ? 1.f : (i % 2 == 1 ? 4.f : 2.f));
			}
			return Sum * h / 3.f;
		}

		/** Fabric coordinates of rest point p in chart Ch. For sleeves u is the angle around the arm
		 * (seam underneath at +-pi) and Rad the distance from the arm axis. */
		static void ChartUV(int Ch, const V3& p, const FRestSkeleton& s, const FCtx& c, float& u, float& v, float& Rad)
		{
			Rad = 0;
			if (Ch == ChFront || Ch == ChBack)
			{
				// arc length around the cross-section from the centre front / back, scaled down
				// where the surface is inside it (collar, hood)
				float a, d;
				c.Radii(p.Y, a, d);
				const float x = p.X / a, z = (Ch == ChFront ? p.Z : -p.Z) / d;
				u = EllipseArc(a, d, FMath::Atan2(x, z)) * FMath::Sqrt(x * x + z * z);
				v = p.Y;
				return;
			}
			if (Ch == ChYoke)
			{
				u = p.X;
				v = s.L + 0.3f + p.Z;
				return;
			}
			const float Side = Ch == ChSleeveL ? -1.f : 1.f;
			const V3 Dir = FRestSkeleton::ArmDir(Side);
			const V3 Rel = p - (Side < 0 ? s.lSh : s.rSh);
			const float Along = Dot(Rel, Dir);
			const V3 Rv = Rel - Dir * Along;
			const V3 Top = Normalized(VUp - Dir * Dot(VUp, Dir));
			u = FMath::Atan2(Dot(Rv, VFwd), Dot(Rv, Top));
			Rad = Rv.Size();
			v = Along;
		}

		/**
		 * Lays the fabric out like sewn pattern pieces, so a 2D texture sits on it with little
		 * stretch: front and back panels unrolled around the torso (side seams), a yoke over the
		 * shoulder tops, and each sleeve unrolled around the arm (seam underneath). UVs are metres
		 * of fabric. Seam vertices are duplicated for rendering; the solver keeps the welded mesh.
		 */
		static void Unwrap(FGarmentMeshData& D, const FRestSkeleton& s, const FGarmentSpec& g, const FCtx& c)
		{
			const TArray<int32>& Tris = D.Tris; const TArray<V3>& R = D.Rest;
			TMap<int64, int32> Map;
			Map.Reserve(R.Num() * 5 / 4);
			D.RenderToSim.Reset(); D.UV.Reset();
			D.RenderTris.SetNumUninitialized(Tris.Num());
			const bool bSleeves = g.Sleeve > 0.f && !g.bTank;
			float Fu[3], Fv[3], Fr[3];
			for (int t = 0; t < Tris.Num(); t += 3)
			{
				const int i0 = Tris[t], i1 = Tris[t + 1], i2 = Tris[t + 2];
				const V3 Cen = (R[i0] + R[i1] + R[i2]) / 3.f;
				const V3 N = Normalized(Cross(R[i1] - R[i0], R[i2] - R[i0]));
				const float Arm = (ArmWeight(D.Weights[i0]) + ArmWeight(D.Weights[i1]) + ArmWeight(D.Weights[i2])) / 3.f;
				bool bOnArm = false;
				if (bSleeves && Arm > 0.5f)
				{
					// not the armpit's body side, which faces along the arm and would smear in a sleeve chart
					const float Sd = Cen.X < 0 ? -1.f : 1.f;
					const V3 Dir = FRestSkeleton::ArmDir(Sd), Rel = Cen - (Sd < 0 ? s.lSh : s.rSh);
					bOnArm = (Rel - Dir * Dot(Rel, Dir)).Size() < 1.5f * (0.056f + g.SleeveEase) * s.u;
				}
				const int Ch = bOnArm ? (Cen.X < 0 ? ChSleeveL : ChSleeveR)
					: N.Y > 0.55f && Cen.Y > 0.8f * s.L ? ChYoke
					: Cen.Z >= 0.f ? ChFront : ChBack;
				for (int k = 0; k < 3; k++) ChartUV(Ch, R[Tris[t + k]], s, c, Fu[k], Fv[k], Fr[k]);
				// sleeves wrap around: a triangle across the seam moves its negative-angle corners one turn on
				const bool bCyl = Ch >= ChSleeveL;
				const bool bAcross = bCyl && FMath::Max3(Fu[0], Fu[1], Fu[2]) - FMath::Min3(Fu[0], Fu[1], Fu[2]) > PI;
				for (int k = 0; k < 3; k++)
				{
					const int Vi = Tris[t + k];
					const int Wrap = bAcross && Fu[k] < 0.f ? 1 : 0;
					const float Uu = bCyl ? (Fu[k] + Wrap * 2.f * PI) * Fr[k] : Fu[k];
					const int64 Key = (int64(Vi) << 4) | (int64(Ch) << 1) | int64(Wrap);
					int32* Found = Map.Find(Key);
					int32 Ri;
					if (Found) Ri = *Found;
					else
					{
						Ri = D.RenderToSim.Num();
						Map.Add(Key, Ri);
						D.RenderToSim.Add(Vi);
						D.UV.Add(V2(Uu, Fv[k]));
					}
					D.RenderTris[t + k] = Ri;
				}
			}
		}

		// ------------------------------------------------------------ ribbed bands
		/**
		 * Each band is where a field (m inside the band) is positive: the collar within RibCollar of
		 * the neck opening, the hem and the cuffs within their width of the edge. The cloth's
		 * triangles are clipped to that region, so the band's edge is a clean line across them.
		 */
		static void BuildRibs(FGarmentMeshData& D, const FRestSkeleton& s, const FGarmentSpec& g, const FCtx& c)
		{
			D.RibVerts.Reset(); D.RibUV.Reset(); D.RibTris.Reset();
			const float u = s.u, wCollar = g.RibCollar * u, wHem = g.RibHem * u, wCuff = g.bTank || g.Sleeve <= 0.f ? 0.f : g.RibCuff * u;
			if (wCollar <= 0.f && wHem <= 0.f && wCuff <= 0.f) return;
			const TArray<V3>& R = D.Rest;
			const int N = R.Num();

			// the neck opening: boundary vertices near the top, between the shoulders
			TArray<V3> Neck;
			if (wCollar > 0.f)
			{
				TMap<int64, int32> EdgeCount;
				for (int t = 0; t < D.Tris.Num(); t += 3)
					for (int e = 0; e < 3; e++) EdgeCount.FindOrAdd(EdgeKey(D.Tris[t + e], D.Tris[t + (e + 1) % 3]))++;
				TSet<int32> Seen;
				for (const auto& kv : EdgeCount)
				{
					if (kv.Value != 1) continue;
					for (const int v : { int(kv.Key >> 32), int(kv.Key & 0xffffffff) })
						if (R[v].Y > s.L - 0.25f * u && FMath::Abs(R[v].X) < s.sw * 0.45f && !Seen.Contains(v)) { Seen.Add(v); Neck.Add(R[v]); }
				}
			}

			// per vertex: how far inside a band (m), and (u, v) around / along that band's axis
			TArray<float> F; TArray<V2> Uv;
			F.SetNumUninitialized(N); Uv.SetNumUninitialized(N);
			float ha, hd;
			c.Radii(c.hemY + 0.5f * wHem, ha, hd);
			for (int i = 0; i < N; i++)
			{
				const V3 p = R[i];
				const float Arm = ArmWeight(D.Weights[i]);
				float Best = -1.f; V2 Q = V2::ZeroVector;
				if (wHem > 0.f && Arm < 0.5f)
				{
					Best = c.hemY + wHem - p.Y;
					Q = V2(FMath::Atan2(p.X, p.Z) * 0.5f * (ha + hd), p.Y);
				}
				if (wCuff > 0.f && Arm > 0.5f)
				{
					const float Side = p.X < 0 ? -1.f : 1.f;
					const V3 End = Side < 0 ? c.lEnd : c.rEnd, Dir = Side < 0 ? c.lDir : c.rDir;
					const float Al = Dot(p - End, Dir), f = wCuff + Al;
					if (f > Best)
					{
						const V3 Rv = (p - End) - Dir * Al, Top = Normalized(VUp - Dir * Dot(VUp, Dir));
						Best = f;
						Q = V2(FMath::Atan2(Dot(Rv, VFwd), Dot(Rv, Top)) * 0.045f * u, Al);
					}
				}
				if (wCollar > 0.f && Neck.Num() > 0 && p.Y > s.L - 0.3f * u)
				{
					float d2 = 1e9f;
					for (const V3& n : Neck) d2 = FMath::Min(d2, V3::DistSquared(p, n));
					const float f = wCollar - FMath::Sqrt(d2);
					if (f > Best)
					{
						Best = f;
						Q = V2(FMath::Atan2(p.X, p.Z + 0.012f * u) * (c.neckR + 0.02f * u), FMath::Sqrt(d2));
					}
				}
				F[i] = Best; Uv[i] = Q;
			}

			// clip each triangle to F > 0; u wraps around the axis, so a triangle across the seam
			// (behind the neck / torso, under the arm) moves its negative side one turn on
			TMap<uint64, int32> Map;
			auto Vert = [&](int a, int b, int Wrap, float Period) -> int32
			{
				if (a > b) Swap(a, b);
				const uint64 Key = (uint64(a) << 33) | (uint64(b) << 2) | uint64(Wrap);
				if (const int32* Found = Map.Find(Key)) return *Found;
				const float t = a == b ? 0.f : F[a] / (F[a] - F[b]);
				auto U = [&](int v) { return V2(Uv[v].X + (Wrap && Uv[v].X < 0.f ? Period : 0.f), Uv[v].Y); };
				D.RibVerts.Add({ a, b, t });
				D.RibUV.Add(U(a) + (U(b) - U(a)) * t);
				return Map.Add(Key, D.RibVerts.Num() - 1);
			};
			for (int t = 0; t < D.Tris.Num(); t += 3)
			{
				const int v[3] = { D.Tris[t], D.Tris[t + 1], D.Tris[t + 2] };
				if (F[v[0]] <= 0.f && F[v[1]] <= 0.f && F[v[2]] <= 0.f) continue;
				const float U0 = FMath::Min3(Uv[v[0]].X, Uv[v[1]].X, Uv[v[2]].X), U1 = FMath::Max3(Uv[v[0]].X, Uv[v[1]].X, Uv[v[2]].X);
				// the seam: a jump in u far larger than a triangle
				float Period = 0.f;
				int Wrap = 0;
				if (U1 - U0 > 6.f * c.Cell) { Wrap = 1; Period = 2.f * FMath::Max(FMath::Abs(U0), FMath::Abs(U1)); }
				int32 Poly[4]; int n = 0;
				for (int e = 0; e < 3; e++)
				{
					const int a = v[e], b = v[(e + 1) % 3];
					if (F[a] > 0.f) Poly[n++] = Vert(a, a, Wrap, Period);
					if ((F[a] > 0.f) != (F[b] > 0.f)) Poly[n++] = Vert(a, b, Wrap, Period);
				}
				if (n >= 3) D.RibTris.Append({ Poly[0], Poly[1], Poly[2] });
				if (n == 4) D.RibTris.Append({ Poly[0], Poly[2], Poly[3] });
			}
		}

		// ------------------------------------------------------------ build
		TSharedPtr<FGarmentMeshData> Build(const FRestSkeleton& s, const FGarmentSpec& g, float CellSize)
		{
			FCtx c;
			c.s = s; c.g = &g; c.Cell = CellSize;
			c.e = g.Ease * s.u; c.hemY = g.Hem * s.L;
			c.neckR = (g.Neck == ENeckline::Crew ? 0.076f : g.Neck == ENeckline::VNeck ? 0.076f : g.Neck == ENeckline::Scoop ? 0.085f : 0.068f) * s.u;
			c.lEnd = SleeveEnd(s, -1, g.Sleeve, c.lDir, c.lLong);
			c.rEnd = SleeveEnd(s, +1, g.Sleeve, c.rDir, c.rLong);
			BuildTables(c);
			const float Cell = CellSize;

			// ---- sample grid
			const float Xr = g.Sleeve > 0.f ? FMath::Max(FMath::Abs(c.lEnd.X), c.rEnd.X) + 0.1f : s.sw * 0.5f + 0.15f;
			const V3 Min(-Xr, FMath::Min(c.hemY, FMath::Min(c.lEnd.Y, c.rEnd.Y)) - 0.1f, -0.25f * s.u);
			const V3 Max(Xr, s.L + 0.15f * s.u, 0.25f * s.u);
			const int nx = FMath::CeilToInt((Max.X - Min.X) / Cell), ny = FMath::CeilToInt((Max.Y - Min.Y) / Cell), nz = FMath::CeilToInt((Max.Z - Min.Z) / Cell);
			const int sx = nx + 1, sy = ny + 1, sz = nz + 1;
			TArray<float> F;
			F.SetNumUninitialized(sx * sy * sz);
			for (int k = 0; k < sz; k++)
				for (int j = 0; j < sy; j++)
					for (int i = 0; i < sx; i++)
						F[i + sx * (j + sy * k)] = Sdf(c, Min + V3(i, j, k) * Cell);

			// ---- surface nets: one vertex per sign-changing cell
			TArray<int32> CellVert;
			CellVert.Init(-1, nx * ny * nz);
			TArray<V3> Verts;
			Verts.Reserve(8192);
			static const int Edges[12][2] = { {0,1},{2,3},{4,5},{6,7},{0,2},{1,3},{4,6},{5,7},{0,4},{1,5},{2,6},{3,7} };
			float Cv[8]; V3 Cp[8];
			for (int k = 0; k < nz; k++)
				for (int j = 0; j < ny; j++)
					for (int i = 0; i < nx; i++)
					{
						int Mask = 0;
						for (int q = 0; q < 8; q++)
						{
							const int di = q & 1, dj = (q >> 1) & 1, dk = (q >> 2) & 1;
							Cv[q] = F[(i + di) + sx * ((j + dj) + sy * (k + dk))];
							Cp[q] = Min + V3(i + di, j + dj, k + dk) * Cell;
							if (Cv[q] < 0) Mask |= 1 << q;
						}
						if (Mask == 0 || Mask == 255) continue;
						V3 Sum = V3::ZeroVector; int n = 0;
						for (int ei = 0; ei < 12; ei++)
						{
							const float a = Cv[Edges[ei][0]], b = Cv[Edges[ei][1]];
							if ((a < 0) == (b < 0)) continue;
							Sum += LerpU(Cp[Edges[ei][0]], Cp[Edges[ei][1]], a / (a - b)); n++;
						}
						CellVert[i + nx * (j + ny * k)] = Verts.Num();
						Verts.Add(Sum / float(n));
					}
			auto CV = [&](int i, int j, int k) { return CellVert[i + nx * (j + ny * k)]; };
			TArray<int32> Quads;
			Quads.Reserve(Verts.Num() * 6);
			for (int k = 1; k < nz; k++)
				for (int j = 1; j < ny; j++)
					for (int i = 1; i < nx; i++)
					{
						const float V0 = F[i + sx * (j + sy * k)];
						// edges leaving corner (i,j,k) along +x, +y, +z
						for (int Axis = 0; Axis < 3; Axis++)
						{
							const int i1 = i + (Axis == 0), j1 = j + (Axis == 1), k1 = k + (Axis == 2);
							if (i1 > nx || j1 > ny || k1 > nz) continue;
							const float V1 = F[i1 + sx * (j1 + sy * k1)];
							if ((V0 < 0) == (V1 < 0)) continue;
							int a, b, cc, d;
							if (Axis == 0) { a = CV(i, j - 1, k - 1); b = CV(i, j, k - 1); cc = CV(i, j, k); d = CV(i, j - 1, k); }
							else if (Axis == 1) { a = CV(i - 1, j, k - 1); b = CV(i - 1, j, k); cc = CV(i, j, k); d = CV(i, j, k - 1); }
							else { a = CV(i - 1, j - 1, k); b = CV(i, j - 1, k); cc = CV(i, j, k); d = CV(i - 1, j, k); }
							if (a < 0 || b < 0 || cc < 0 || d < 0) continue;
							if (V0 < 0) Quads.Append({ a, b, cc, d });
							else Quads.Append({ a, d, cc, b });
						}
					}

			// ---- project vertices onto the true surface
			for (V3& Vtx : Verts)
			{
				V3 p = Vtx;
				for (int It = 0; It < 3; It++)
				{
					const V3 Gr = Grad(c, p);
					const float Gg = Gr.SizeSquared();
					if (Gg < 1e-8f) break;
					p -= Gr * (Sdf(c, p) / Gg);
				}
				if ((p - Vtx).SizeSquared() < Cell * Cell * 2.f) Vtx = p;
			}

			// ---- triangulate quads (shorter diagonal) and orient outward
			TArray<int32> Tris;
			Tris.Reserve(Quads.Num() / 4 * 6);
			for (int q = 0; q < Quads.Num(); q += 4)
			{
				const int a = Quads[q], b = Quads[q + 1], cc = Quads[q + 2], d = Quads[q + 3];
				if ((Verts[a] - Verts[cc]).SizeSquared() < (Verts[b] - Verts[d]).SizeSquared()) Tris.Append({ a, b, cc, a, cc, d });
				else Tris.Append({ a, b, d, b, cc, d });
			}
			float Outward = 0;
			for (int t = 0; t < Tris.Num(); t += 3)
			{
				const V3 p0 = Verts[Tris[t]];
				Outward += Dot(Cross(Verts[Tris[t + 1]] - p0, Verts[Tris[t + 2]] - p0), Grad(c, p0));
			}
			if (Outward < 0) for (int t = 0; t < Tris.Num(); t += 3) Swap(Tris[t + 1], Tris[t + 2]);

			// ---- cut openings: remove the neck floor, hem cap and cuff caps
			TArray<int32> Keep;
			Keep.Reserve(Tris.Num());
			// crew and scoop: the opening dips in front, a parabola ScoopDepth deep and 2 ScoopW wide
			const bool bScoop = g.Neck == ENeckline::Scoop, bCrew = g.Neck == ENeckline::Crew;
			const float VDepth = 0.15f * s.u, ScoopDepth = (bScoop ? 0.12f : 0.042f) * s.u, ScoopW = bScoop ? s.sw * 0.2f : c.neckR;
			for (int t = 0; t < Tris.Num(); t += 3)
			{
				const V3 p0 = Verts[Tris[t]], p1 = Verts[Tris[t + 1]], p2 = Verts[Tris[t + 2]];
				const V3 Cen = (p0 + p1 + p2) / 3.f, N = Normalized(Cross(p1 - p0, p2 - p0));
				bool bCut = false;
				if (Cen.Y < c.hemY + Cell * 0.4f && N.Y < -0.6f) bCut = true;                                   // hem
				const float RXZ = V2(Cen.X, Cen.Z + 0.012f * s.u).Size();
				if (RXZ < c.neckR * 0.98f && Cen.Y > s.L - 0.08f * s.u && N.Y > 0.25f) bCut = true;             // neck floor
				if (g.Neck == ENeckline::VNeck && Cen.Z > 0 && N.Z > 0.2f && Cen.Y > s.L - VDepth
					&& FMath::Abs(Cen.X) < (Cen.Y - (s.L - VDepth)) * 0.42f + 0.005f) bCut = true;              // V
				if ((bScoop || bCrew) && Cen.Z > 0 && N.Z > 0.2f && FMath::Abs(Cen.X) < ScoopW
					&& Cen.Y > s.L - ScoopDepth * (1.f - FMath::Square(Cen.X / ScoopW))) bCut = true;          // scoop
				if (g.bTank && N.Y > 0.6f && Cen.Y > s.L - 0.075f * s.u - Cell * 0.6f && Cen.Y < s.L - 0.04f * s.u) bCut = true; // armholes
				if (g.Sleeve > 0.f && !g.bTank)
				{
					if (Dot(Cen - c.lEnd, c.lDir) > -Cell * 0.4f && Dot(N, c.lDir) > 0.6f) bCut = true;
					if (Dot(Cen - c.rEnd, c.rDir) > -Cell * 0.4f && Dot(N, c.rDir) > 0.6f) bCut = true;
				}
				if (!bCut) Keep.Append({ Tris[t], Tris[t + 1], Tris[t + 2] });
			}

			TSharedPtr<FGarmentMeshData> D = MakeShared<FGarmentMeshData>();
			Compact(Verts, KeepLargestComponent(Keep, Verts.Num()), *D);
			SmoothBoundaries(*D);
			Normals(D->Rest, D->Tris, D->RestN);
			Skin(*D, s, g, c);
			Unwrap(*D, s, g, c);
			BuildRibs(*D, s, g, c);
			return D;
		}
	}
}
