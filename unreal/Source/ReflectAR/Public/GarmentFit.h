#pragma once

#include "GarmentAsset.h"
#include "GarmentMesher.h"

namespace ReflectAR
{
	/** The garment mesh of that name (GarmentAsset.h), loaded once and kept. Null if it isn't in the
	 * project (the garment is then generated procedurally). Game thread. */
	TSharedPtr<const FGarmentAsset> FindGarmentAsset(const FString& Name);

	/**
	 * Fits an authored garment, draped on the reference body, to the user's body.
	 *
	 * The torso part is reshaped section by section: at each height the reference body's ellipse
	 * becomes the user's measured one (FRestSkeleton::TorsoRadii: width and depth from the depth
	 * camera), and every vertex keeps its place around the body and its distance off it. So the
	 * ease the garment was cut with stays the same in centimetres: on a slender build the fabric
	 * closes in around a small section, on a heavy one it is carried out around the belly, with
	 * its folds intact. Sleeves are scaled along the user's arm lengths and move with the
	 * shoulders; the two are blended across the armhole. Heights scale with the torso length.
	 *
	 * The result is what the cloth solver takes from GarmentMesher: rest shape, skin weights,
	 * looseness (from the mesh's pin weights), elastic bands, UVs and the ribbed bands.
	 * Thread-safe.
	 */
	TSharedPtr<FGarmentMeshData> FitGarmentAsset(const FGarmentAsset& A, const FRestSkeleton& S);
}
