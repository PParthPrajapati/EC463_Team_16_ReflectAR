#pragma once

#include "BodyMath.h"
#include "UObject/StrongObjectPtr.h"

class UTexture2D;

namespace ReflectAR
{
	enum class EPattern : uint8 { Solid, Stripes, Plaid, Quilt, Denim, Knit, Pique, Rib };
	enum class ENeckline : uint8 { Crew, VNeck, Collar, HoodDown, Scoop };

	/**
	 * One upper-body garment: its cut (used to generate a seamless mesh), its fabric (used by the
	 * XPBD cloth solver) and its look. Lengths are in metres for a 0.36 m shoulder-joint span and
	 * scale with the user's body.
	 */
	struct FGarmentSpec
	{
		FString Name = TEXT("Garment");
		FString Detail;                   // one short line for the UI: fabric and cut
		/** The authored mesh this garment is made from (garments/<Asset>.fbx, see GarmentAsset.h). The
		 *  cut below is then only the fallback for when that mesh isn't in the project. */
		FString Asset;
		FLinearColor ColorA = FLinearColor::White, ColorB = FLinearColor::Gray;   // sRGB-authored colours
		EPattern Pattern = EPattern::Solid;
		float Tile = 0.25f;               // metres per pattern repeat

		// ---- cut
		float Hem = -0.08f;               // hem height, fraction of torso length (0 = hip joints)
		float Ease = 0.035f;              // room around the torso (m of radius): what the fabric drapes with
		float Sleeve = 0.3f;              // fraction of the arm covered (0 = sleeveless)
		float SleeveEase = 0.024f;
		ENeckline Neck = ENeckline::Crew;
		bool bTank = false;               // narrow straps instead of shoulders
		float Quilt = 0.f;                // puffer baffle bulge height (m)
		float CuffFlare = 0.f;            // extra radius at the sleeve opening
		int Placket = 0;                  // front closure: 0 none, 1 buttons, 2 zipper
		float PlacketLen = 1.f;           // fraction of the front it runs down (polo = short)

		// ---- fabric (XPBD compliances, per s^2)
		float Stretch = 2e-7f;            // stretch compliance (lower = less stretchy)
		float Compress = 6e-6f;           // compression compliance (higher = shrinks before it folds)
		float Bend = 4e-5f;               // bending compliance (higher = softer, drapier)
		float Damping = 0.04f;
		float Drape = 1.f;                // how far free cloth may swing from where the body carries it
		float Sheen = 0.25f;              // cloth shading model: fuzz / sheen amount
		float Density = 1.f;              // relative weight (gravity sag)

		// ---- fit: how the cut follows the measured body (GarmentMesher)
		float Fit = 0.7f;                 // below the chest: 0 = follows the waist, 1 = hangs straight
		float HemBand = 0.f;              // ribbed hem band pulled in to the hips (0..1)
		bool bCuffBand = false;           // gathered cuffs at the wrist (long sleeves)
		float AssetEase = 0.6f;           // share of an authored mesh's room off the body that is kept (1 = as authored, relaxed)
		float Slack = 0.02f;              // extra fabric between where the garment is held: what bunches into folds

		// ---- ribbed knit bands, drawn as a second layer on the cloth (widths in m, 0 = none)
		float RibCollar = 0.f, RibHem = 0.f, RibCuff = 0.f;
		FLinearColor RibColor = FLinearColor(-1.f, 0.f, 0.f);   // r < 0: the garment's own colour

		// ---- material and trims
		float Smooth = 0.22f;             // perceptual smoothness of the fibres (roughness = 1 - this)
		float Bump = 1.f;                 // weave normal-map strength
		float Fold = 1.f;                 // soft drape folds and creases in the shading (stiff fabrics: less)
		float Heather = 0.1f;             // fibre-to-fibre colour variation (heather grey: high)
		float Cloth = 1.f;                // 1 = matte fibre response (cotton, wool), lower = smoother shell fabrics (nylon, satin)
		FLinearColor Trim = FLinearColor(0.92f, 0.9f, 0.85f);    // buttons, or the zipper tape
		FLinearColor Metal = FLinearColor(0.74f, 0.74f, 0.76f);  // zipper teeth / slider, metal buttons
		bool bMetalButtons = false;
	};

	/** The upper-body garments in the catalogue. */
	const TArray<FGarmentSpec>& GarmentCatalog();

	/** Colour and weave height (0..1) at (u, v) in [0, 1) of one repeat of the pattern. */
	void SamplePattern(const FGarmentSpec& S, float u, float v, FLinearColor& OutColor, float& OutHeight);

	/** Noise in [0, 1] that repeats every Period cells in x and y (smooth value noise). */
	float TileNoise(float x, float y, int Period, int Seed = 0);

	/**
	 * Pixels of the material maps for one repeat of a fabric, N x N, row-major: base colour (linear
	 * values, stored as sRGB), tangent-space normal from the weave height, and a mask (G = ambient
	 * occlusion in the weave's valleys, A = smoothness).
	 */
	void BuildFabricPixels(const FGarmentSpec& S, int N, TArray<FLinearColor>& Col, TArray<FLinearColor>& Nrm, TArray<FLinearColor>& Msk);

	/**
	 * Pixels of the drape map shared by all garments, one repeat of FoldTileM metres: RG = normal of
	 * soft folds a few centimetres wide, B = their height (0.5 = flat), A = slow variation of the
	 * dye and wear, used to keep the fabric's colour from being perfectly flat.
	 */
	constexpr float FoldTileM = 0.64f;
	void BuildFoldPixels(int N, TArray<FLinearColor>& Out);

	/**
	 * Material maps for one repeat of a fabric: base colour (sRGB), tangent-space normal map from
	 * the weave height (linear), and a mask (linear: G = ambient occlusion in the weave's valleys,
	 * A = smoothness). Game thread only.
	 */
	struct FFabricMaps
	{
		TStrongObjectPtr<UTexture2D> BaseColor, Normal, Mask;
		static const FFabricMaps& Get(const FGarmentSpec& S);
		/** The drape map (BuildFoldPixels), shared by all garments. */
		static UTexture2D* Fold();
		/** The maps of the garment's ribbed bands. */
		static const FFabricMaps& GetRib(const FGarmentSpec& S);
	};
}
