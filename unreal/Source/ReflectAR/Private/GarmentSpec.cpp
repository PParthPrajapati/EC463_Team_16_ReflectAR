#include "GarmentSpec.h"
#include "Engine/Texture2D.h"
#include "TextureResource.h"

namespace ReflectAR
{
	static FLinearColor C(const TCHAR* Hex) { return FLinearColor(FColor::FromHex(Hex)); }

	const TArray<FGarmentSpec>& GarmentCatalog()
	{
		static TArray<FGarmentSpec> All = []
		{
			TArray<FGarmentSpec> A;
			auto Add = [&A](TFunctionRef<void(FGarmentSpec&)> Init) { FGarmentSpec S; Init(S); A.Add(S); };
			// Compliances are per second squared: at 8 substeps of a 60 Hz frame, 4e-6 is "about as
			// stiff as the constraint itself". Compress < Bend, so slack folds rather than shrinks.
			Add([](FGarmentSpec& g) { g.Name = TEXT("Heavyweight Tee"); g.Asset = TEXT("heavyweight_tee"); g.Detail = TEXT("Boxy fit  ·  240 gsm cotton jersey");
				g.ColorA = C(TEXT("EDEBE6")); g.Pattern = EPattern::Knit; g.Sleeve = 0.36f; g.Hem = -0.16f;
				g.Ease = 0.034f; g.SleeveEase = 0.024f; g.CuffFlare = 0.004f; g.Fit = 1.f; g.Slack = 0.015f; g.RibCollar = 0.02f;
				g.Compress = 6e-6f; g.Bend = 4e-5f; g.Damping = 0.06f; g.Smooth = 0.18f; g.Tile = 0.125f; g.Heather = 0.12f; });
			Add([](FGarmentSpec& g) { g.Name = TEXT("Crewneck Sweatshirt"); g.Asset = TEXT("crewneck_sweatshirt"); g.Detail = TEXT("Relaxed fit  ·  brushed cotton fleece");
				g.ColorA = C(TEXT("8D9199")); g.Pattern = EPattern::Knit; g.Sleeve = 0.98f; g.Hem = -0.1f;
				g.Ease = 0.045f; g.SleeveEase = 0.028f; g.Fit = 1.f; g.HemBand = 1.f; g.bCuffBand = true; g.Slack = 0.05f;
				g.RibCollar = 0.024f; g.RibHem = 0.06f; g.RibCuff = 0.06f;
				g.Compress = 1e-5f; g.Bend = 1.6e-5f; g.Damping = 0.08f; g.Density = 1.15f; g.Smooth = 0.08f; g.Bump = 0.8f; g.Tile = 0.16f; g.Sheen = 0.4f; g.Heather = 0.85f; g.Fold = 0.8f; });
			Add([](FGarmentSpec& g) { g.Name = TEXT("Satin Blouse"); g.Asset = TEXT("satin_blouse"); g.Detail = TEXT("Fluid drape  ·  washed silk satin");
				g.ColorA = C(TEXT("6E1F33")); g.Pattern = EPattern::Solid; g.Sleeve = 0.95f; g.Hem = -0.2f; g.Neck = ENeckline::VNeck;
				g.Ease = 0.04f; g.SleeveEase = 0.022f; g.CuffFlare = 0.012f; g.Fit = 1.f; g.Drape = 1.3f; g.Slack = 0.06f;
				g.Compress = 2e-6f; g.Bend = 4e-4f; g.Damping = 0.04f; g.Sheen = 0.7f; g.Smooth = 0.62f; g.Bump = 0.25f; g.Tile = 0.08f; g.Cloth = 0.3f; g.Heather = 0.03f; });
			Add([](FGarmentSpec& g) { g.Name = TEXT("Flannel Overshirt"); g.Asset = TEXT("flannel_overshirt"); g.Detail = TEXT("Straight fit  ·  brushed cotton twill");
				g.ColorA = C(TEXT("A3262A")); g.ColorB = C(TEXT("1B1B1B")); g.Pattern = EPattern::Plaid; g.Tile = 0.16f;
				g.Sleeve = 0.97f; g.Hem = -0.24f; g.Neck = ENeckline::Collar; g.Placket = 1;
				g.Ease = 0.045f; g.SleeveEase = 0.026f; g.CuffFlare = 0.004f; g.Fit = 1.f;
				g.Compress = 6e-6f; g.Bend = 2.5e-5f; g.Damping = 0.06f; g.Smooth = 0.12f; g.Bump = 0.7f; g.Sheen = 0.4f; g.Heather = 0.25f; g.Trim = C(TEXT("EDE6D6")); });
			Add([](FGarmentSpec& g) { g.Name = TEXT("Bomber Jacket"); g.Asset = TEXT("bomber_jacket"); g.Detail = TEXT("Ribbed trims  ·  matte nylon twill");
				g.ColorA = C(TEXT("4B5B3A")); g.Pattern = EPattern::Solid; g.Sleeve = 0.98f; g.Hem = -0.04f; g.Neck = ENeckline::Collar; g.Placket = 2;
				g.Ease = 0.055f; g.SleeveEase = 0.034f; g.Fit = 1.f; g.HemBand = 1.f; g.bCuffBand = true; g.Slack = 0.05f;
				g.RibCollar = 0.1f; g.RibHem = 0.06f; g.RibCuff = 0.06f; g.RibColor = C(TEXT("2B3322"));
				g.Compress = 4e-6f; g.Bend = 8e-6f; g.Damping = 0.08f; g.Density = 1.2f; g.Sheen = 0.45f; g.Smooth = 0.45f; g.Bump = 0.5f; g.Tile = 0.1f; g.Cloth = 0.35f; g.Fold = 0.7f; g.Heather = 0.03f;
				g.Trim = C(TEXT("2A3024")); g.Metal = C(TEXT("8E9096")); });
			return A;
		}();
		return All;
	}

	static float Rep(float x) { return x - FMath::FloorToFloat(x); }

	// ---------------------------------------------------------------- tileable noise
	static uint32 Hash(int32 x, int32 y, int32 s)
	{
		uint32 h = uint32(x) * 374761393u + uint32(y) * 668265263u + uint32(s) * 2246822519u;
		h = (h ^ (h >> 13)) * 1274126177u;
		return h ^ (h >> 16);
	}

	float TileNoise(float x, float y, int Period, int Seed)
	{
		const int x0 = FMath::FloorToInt(x), y0 = FMath::FloorToInt(y);
		const float fx = x - x0, fy = y - y0;
		const float ux = fx * fx * (3.f - 2.f * fx), uy = fy * fy * (3.f - 2.f * fy);
		auto At = [Period, Seed](int ix, int iy)
		{
			ix %= Period; if (ix < 0) ix += Period;
			iy %= Period; if (iy < 0) iy += Period;
			return float(Hash(ix, iy, Seed) & 0xffff) / 65535.f;
		};
		const float a = At(x0, y0), b = At(x0 + 1, y0), c = At(x0, y0 + 1), d = At(x0 + 1, y0 + 1);
		return a + (b - a) * ux + (c - a) * uy + (a - b - c + d) * ux * uy;
	}

	/** A few octaves of TileNoise, [0, 1], repeating once over u, v in [0, 1). */
	static float Fbm(float u, float v, int Cells, int Octaves, int Seed)
	{
		float Sum = 0.f, Amp = 0.5f, Norm = 0.f;
		for (int o = 0; o < Octaves; o++, Cells *= 2, Amp *= 0.5f)
		{
			Sum += TileNoise(u * Cells, v * Cells, Cells, Seed + o * 17) * Amp;
			Norm += Amp;
		}
		return Sum / Norm;
	}

	// ---------------------------------------------------------------- weaves
	// Every pattern is built from whole numbers of yarns per repeat, so it tiles without a seam,
	// at about life size: 1.5-2 mm stitches for jersey, 0.4-0.6 mm threads for woven cloth (which
	// a 512 px repeat only resolves as texture, not as individual threads).

	/** Jersey: columns (wales) of V-shaped stitches. Returns height; Yarn = which stitch, for colour. */
	static float Jersey(float u, float v, int Wales, int Courses, float& Yarn)
	{
		const float a = u * Wales, cu = Rep(a);
		const float Leg = FMath::Abs(cu - 0.5f) * 2.f;                 // 0 at the middle of the wale, 1 between wales
		const float b = v * Courses + Leg * 0.5f, cv = Rep(b);         // the two legs of the V rise toward the centre
		Yarn = float(Hash(FMath::FloorToInt(a), FMath::FloorToInt(b), 3) & 0xff) / 255.f;
		const float Across = FMath::Cos((Leg - 0.45f) * PI * 0.9f);    // each leg is a round yarn
		const float Along = 0.5f + 0.5f * FMath::Cos((cv - 0.5f) * 2.f * PI);
		return Clamp01(0.15f + 0.85f * Clamp01(Across) * (0.55f + 0.45f * Along));
	}

	/** Plain weave: warp and weft alternate over / under. */
	static float PlainWeave(float u, float v, int Threads, float& Yarn)
	{
		const float a = u * Threads, b = v * Threads;
		const int ia = FMath::FloorToInt(a), ib = FMath::FloorToInt(b);
		const bool bWarpUp = ((ia + ib) & 1) == 0;
		const float fa = Rep(a) - 0.5f, fb = Rep(b) - 0.5f;
		// the thread on top is a ridge along its own direction, bowed where it dives under
		const float Ridge = bWarpUp ? FMath::Cos(fa * PI) * (0.6f + 0.4f * FMath::Cos(fb * PI)) : FMath::Cos(fb * PI) * (0.6f + 0.4f * FMath::Cos(fa * PI));
		Yarn = float(Hash(bWarpUp ? ia : 7919, bWarpUp ? 104729 : ib, 5) & 0xff) / 255.f;
		return Clamp01(0.2f + 0.8f * Ridge);
	}

	/** Twill: each weft passes over Over warps, stepping one thread per row: diagonal wales.
	 * bWarp = the warp is on top here (denim: the dyed yarn). */
	static float Twill(float u, float v, int Threads, int Over, int Under, bool& bWarp, float& Yarn)
	{
		const float a = u * Threads, b = v * Threads;
		const int ia = FMath::FloorToInt(a), ib = FMath::FloorToInt(b), n = Over + Under;
		int k = (ia + ib) % n; if (k < 0) k += n;
		bWarp = k < Over;
		const float fa = Rep(a) - 0.5f, fb = Rep(b) - 0.5f;
		const float Ridge = bWarp ? FMath::Cos(fa * PI) * (0.75f + 0.25f * FMath::Cos(fb * PI)) : FMath::Cos(fb * PI) * (0.75f + 0.25f * FMath::Cos(fa * PI));
		Yarn = float(Hash(bWarp ? ia : 7919, bWarp ? 104729 : ib, 9) & 0xff) / 255.f;
		return Clamp01((bWarp ? 0.35f : 0.1f) + 0.65f * Ridge);
	}

	void SamplePattern(const FGarmentSpec& S, float u, float v, FLinearColor& c, float& h)
	{
		c = S.ColorA;
		h = 0.5f;
		float Yarn = 0.5f;
		switch (S.Pattern)
		{
		case EPattern::Stripes:
			c = v < 0.5f ? S.ColorA : S.ColorB;
			h = Jersey(u, v, 40, 52, Yarn);
			break;
		case EPattern::Plaid:
		{
			// yarn-dyed twill: the colour of a point is that of whichever thread is on top
			const float bu = Rep(u) < 0.38f ? 1.f : 0.f, bv = Rep(v) < 0.38f ? 1.f : 0.f;
			const float Tu = FMath::Abs(u - 0.7f) < 0.025f ? 1.f : 0.f, Tv = FMath::Abs(v - 0.7f) < 0.025f ? 1.f : 0.f;
			bool bWarp;
			h = Twill(u, v, 128, 2, 2, bWarp, Yarn);
			const float Band = bWarp ? bu : bv, Thin = bWarp ? Tu : Tv;
			c = FMath::Lerp(S.ColorA, S.ColorB, Band);
			c = FMath::Lerp(c, FLinearColor(0.9f, 0.85f, 0.8f), Thin * 0.6f);
			// brushed: the nap blurs the two thread colours into each other
			c = FMath::Lerp(c, FMath::Lerp(S.ColorA, S.ColorB, 0.5f * (bu + bv)), 0.55f);
			break;
		}
		case EPattern::Quilt:
		{
			const float d = FMath::Abs(v - 0.5f) * 2.f;       // one baffle per repeat, seam at the edges
			float Y2;
			const float Weave = PlainWeave(u, v, 128, Y2);     // fine nylon taffeta
			const float Seam = FMath::Pow(d, 10.f);
			h = FMath::Sqrt(FMath::Max(1.f - d * d * d * d, 0.f)) * 0.85f + 0.06f * Weave - 0.15f * Seam;
			// gathers radiating from the stitched seam
			h += 0.05f * Seam * FMath::Sin(u * 2.f * PI * 37.f + 3.f * TileNoise(u * 16.f, 0.f, 16, 2));
			c = S.ColorA * (1.f - 0.5f * FMath::Pow(d, 16.f));
			Yarn = Y2;
			break;
		}
		case EPattern::Denim:
		{
			// 3/1 twill: indigo warp over white weft, with slubs (uneven yarn) running along the warp
			bool bWarp;
			h = Twill(u, v, 96, 3, 1, bWarp, Yarn);
			const float Slub = TileNoise(u * 96.f, v * 12.f, 96, 4) * 0.6f + TileNoise(u * 96.f, v * 3.f, 96, 6) * 0.4f;
			const FLinearColor Indigo = S.ColorA * (0.7f + 0.5f * Slub), Weft = FMath::Lerp(S.ColorA, FLinearColor::White, 0.6f);
			c = bWarp ? Indigo : FMath::Lerp(Indigo, Weft, 0.5f);
			h = Clamp01(h + (Slub - 0.5f) * 0.25f);
			break;
		}
		case EPattern::Knit:
			h = Jersey(u, v, 64, 84, Yarn);
			break;
		case EPattern::Pique:
		{
			// honeycomb of raised cells, offset every other row
			const float b = v * 44.f, a = u * 40.f + ((FMath::FloorToInt(b) & 1) ? 0.5f : 0.f);
			const float cu = Rep(a) - 0.5f, cv = Rep(b) - 0.5f;
			h = Clamp01(1.15f - (cu * cu + cv * cv) * 5.5f);
			float Y2;
			h = h * (0.8f + 0.2f * Jersey(u, v, 160, 176, Y2));
			Yarn = float(Hash(FMath::FloorToInt(a), FMath::FloorToInt(b), 11) & 0xff) / 255.f;
			break;
		}
		case EPattern::Rib:
		{
			// 2x2 rib: bands of stitches standing forward and sitting back
			const float Band = 0.5f + 0.5f * FMath::Cos(u * 2.f * PI * 24.f);
			h = Jersey(u, v, 96, 110, Yarn) * 0.45f + 0.55f * FMath::Pow(Band, 0.6f);
			c = S.ColorA * (0.82f + 0.18f * Band);
			break;
		}
		default:                                              // smooth shell fabric: a fine plain weave
			h = 0.35f + 0.3f * PlainWeave(u, v, 160, Yarn);
			break;
		}
		// yarn in the valleys is in shadow; every yarn takes the dye slightly differently; fibres
		// of another shade are spun in (heather)
		const float Fibre = TileNoise(u * 256.f, v * 96.f, 256, 21) * 0.6f + TileNoise(u * 96.f, v * 256.f, 256, 23) * 0.4f;
		const float Shade = (0.86f + 0.14f * h) * (1.f + (Yarn - 0.5f) * (0.05f + 0.1f * S.Heather));
		const float Fleck = (Fibre - 0.5f) * 2.f * S.Heather;
		c = c * (Shade * (1.f + 0.22f * Fleck));
		if (S.Heather > 0.3f)                                    // the odd pale fibre
			c = FMath::Lerp(c, FLinearColor(0.9f, 0.9f, 0.9f), Clamp01((Fibre - 0.72f) * 6.f) * 0.3f * S.Heather);
		c.A = 1.f;
	}

	void BuildFabricPixels(const FGarmentSpec& S, int N, TArray<FLinearColor>& Col, TArray<FLinearColor>& Nrm, TArray<FLinearColor>& Msk)
	{
		TArray<float> H;
		Col.SetNumUninitialized(N * N); Nrm.SetNumUninitialized(N * N); Msk.SetNumUninitialized(N * N); H.SetNumUninitialized(N * N);
		for (int y = 0; y < N; y++)
			for (int x = 0; x < N; x++)
				SamplePattern(S, (x + 0.5f) / N, (y + 0.5f) / N, Col[y * N + x], H[y * N + x]);
		// slope per texel of height: weaves are a fraction of a millimetre high over a few texels
		const float k = 2.2f * S.Bump;
		auto Ht = [&](int i, int j) { return H[((j + N) % N) * N + (i + N) % N]; };
		for (int y = 0; y < N; y++)
			for (int x = 0; x < N; x++)
			{
				const float dx = (Ht(x + 1, y) - Ht(x - 1, y)) * 0.5f * k, dy = (Ht(x, y + 1) - Ht(x, y - 1)) * 0.5f * k;
				const V3 n = Normalized(V3(-dx, -dy, 1.f));
				Nrm[y * N + x] = FLinearColor(n.X * 0.5f + 0.5f, n.Y * 0.5f + 0.5f, n.Z * 0.5f + 0.5f, 1.f);
				const float hv = Ht(x, y);
				Msk[y * N + x] = FLinearColor(0.f, FMath::Lerp(0.6f, 1.f, hv), 0.f, Clamp01(S.Smooth * FMath::Lerp(0.75f, 1.15f, hv)));
				FLinearColor& c = Col[y * N + x];
				c = FLinearColor(Clamp01(c.R), Clamp01(c.G), Clamp01(c.B), 1.f);
			}
	}

	// ---------------------------------------------------------------- drape folds
	void BuildFoldPixels(int N, TArray<FLinearColor>& Out)
	{
		// Height field of soft folds: a few families of ridges that run mostly down the garment
		// (whole numbers of waves per repeat, so it tiles), bent by slow noise so that no two look
		// alike, plus broad dents. In metres, for a repeat of FoldTileM.
		struct FWave { int fu, fv; float Amp, Sharp; int Seed; };
		static const FWave Waves[] =
		{
			{ 3, 1, 0.0075f, 1.6f, 31 }, { 5, -1, 0.0050f, 2.2f, 37 }, { 2, 2, 0.0055f, 1.3f, 41 },
			{ 8, 1, 0.0025f, 2.6f, 43 }, { 1, 4, 0.0030f, 1.8f, 47 }, { 11, -3, 0.0015f, 2.0f, 53 },
		};
		TArray<float> H;
		H.SetNumUninitialized(N * N);
		for (int y = 0; y < N; y++)
			for (int x = 0; x < N; x++)
			{
				const float u = (x + 0.5f) / N, v = (y + 0.5f) / N;
				float h = 0.f;
				for (const FWave& W : Waves)
				{
					const float Warp = (Fbm(u, v, 2, 3, W.Seed) - 0.5f) * 5.f;
					const float Where = Fbm(u, v, 3, 2, W.Seed + 5);                          // folds come and go
					const float s = 0.5f + 0.5f * FMath::Sin(2.f * PI * (W.fu * u + W.fv * v) + Warp);
					h += W.Amp * FMath::Pow(s, W.Sharp) * Clamp01(Where * 2.2f - 0.45f);
				}
				h += 0.0025f * Fbm(u, v, 4, 3, 71);
				H[y * N + x] = h;
			}
		Out.SetNumUninitialized(N * N);
		const float Texel = FoldTileM / N;
		auto Ht = [&](int i, int j) { return H[((j + N) % N) * N + (i + N) % N]; };
		for (int y = 0; y < N; y++)
			for (int x = 0; x < N; x++)
			{
				const float dx = (Ht(x + 2, y) - Ht(x - 2, y)) / (4.f * Texel), dy = (Ht(x, y + 2) - Ht(x, y - 2)) / (4.f * Texel);
				const V3 n = Normalized(V3(-dx, -dy, 1.f));
				const float u = (x + 0.5f) / N, v = (y + 0.5f) / N;
				const float Dye = 0.8f * Fbm(u, v, 2, 3, 83) + 0.2f * Fbm(u, v, 12, 2, 89);
				Out[y * N + x] = FLinearColor(n.X * 0.5f + 0.5f, n.Y * 0.5f + 0.5f, Clamp01(0.5f + (Ht(x, y) - 0.01f) * 25.f), Clamp01(Dye));
			}
	}

	/** Repeat-wrapped 8-bit BGRA texture with a full box-filtered mip chain. */
	static UTexture2D* MakeTexture(const TArray<FLinearColor>& Px, int N, bool bSRGB, const TCHAR* Name)
	{
		UTexture2D* T = UTexture2D::CreateTransient(N, N, PF_B8G8R8A8, FName(Name));
		T->SRGB = bSRGB;
		T->AddressX = T->AddressY = TA_Wrap;
		T->Filter = TF_Trilinear;
		T->NeverStream = true;
		FTexturePlatformData* PD = T->GetPlatformData();

		TArray<FLinearColor> Level = Px;
		int Size = N;
		for (int Mip = 0; Size >= 1; Mip++)
		{
			FTexture2DMipMap* M;
			if (Mip == 0) M = &PD->Mips[0];
			else { M = new FTexture2DMipMap(Size, Size, 1); PD->Mips.Add(M); }
			M->SizeX = M->SizeY = Size;
			M->BulkData.Lock(LOCK_READ_WRITE);
			FColor* Dst = reinterpret_cast<FColor*>(M->BulkData.Realloc(int64(Size) * Size * sizeof(FColor)));
			for (int i = 0; i < Size * Size; i++) Dst[i] = Level[i].ToFColor(bSRGB);     // FColor is BGRA in memory
			M->BulkData.Unlock();
			if (Size == 1) break;
			const int H = Size / 2;
			TArray<FLinearColor> Next;
			Next.SetNumUninitialized(H * H);
			for (int y = 0; y < H; y++)
				for (int x = 0; x < H; x++)
					Next[y * H + x] = (Level[(2 * y) * Size + 2 * x] + Level[(2 * y) * Size + 2 * x + 1]
						+ Level[(2 * y + 1) * Size + 2 * x] + Level[(2 * y + 1) * Size + 2 * x + 1]) * 0.25f;
			Level = MoveTemp(Next);
			Size = H;
		}
		T->UpdateResource();
		return T;
	}

	const FFabricMaps& FFabricMaps::GetRib(const FGarmentSpec& S)
	{
		FGarmentSpec R = S;
		R.Name = S.Name + TEXT(" Rib");
		R.Pattern = EPattern::Rib;
		if (S.RibColor.R >= 0.f) R.ColorA = S.RibColor;
		R.Bump = 1.4f;
		R.Heather = FMath::Min(S.Heather, 0.2f);
		R.Smooth = FMath::Min(S.Smooth, 0.2f);
		return Get(R);
	}

	const FFabricMaps& FFabricMaps::Get(const FGarmentSpec& S)
	{
		// never destroyed: a static's destructor would release the textures after UObjects are gone
		static TMap<FString, FFabricMaps>& Cache = *new TMap<FString, FFabricMaps>();
		if (FFabricMaps* Found = Cache.Find(S.Name)) return *Found;
		constexpr int N = 512;
		TArray<FLinearColor> Col, Nrm, Msk;
		BuildFabricPixels(S, N, Col, Nrm, Msk);
		FFabricMaps M;
		M.BaseColor.Reset(MakeTexture(Col, N, true, *(S.Name + TEXT("_BaseColor"))));
		M.Normal.Reset(MakeTexture(Nrm, N, false, *(S.Name + TEXT("_Normal"))));
		M.Mask.Reset(MakeTexture(Msk, N, false, *(S.Name + TEXT("_Mask"))));
		return Cache.Add(S.Name, MoveTemp(M));
	}

	UTexture2D* FFabricMaps::Fold()
	{
		static TStrongObjectPtr<UTexture2D>& Tex = *new TStrongObjectPtr<UTexture2D>();     // never destroyed, like the cache above
		if (!Tex.IsValid())
		{
			constexpr int N = 512;
			TArray<FLinearColor> Px;
			BuildFoldPixels(N, Px);
			Tex.Reset(MakeTexture(Px, N, false, TEXT("ReflectAR_Fold")));
		}
		return Tex.Get();
	}
}
