#pragma once

#include "BodyRig.h"
#include "GarmentSpec.h"

namespace ReflectAR
{
	/** Canonical rest pose built from the user's measurements: hips at the origin, +X = user's
	 * right, +Y up, +Z = toward the mirror, arms 40 degrees below horizontal. */
	struct FRestSkeleton
	{
		float L = 0.5f, sw = 0.36f, hipW = 0.22f, u = 1.f, upperL = 0.29f, upperR = 0.29f, foreL = 0.26f, foreR = 0.26f;
		V3 hipC, shC, lSh, rSh, lEl, rEl, lWr, rWr;
		float Profile[6] = { 1, 1, 1, 1, 1, 1 };    // measured torso widths (FBodyRig::Profile), copied
		float DepthProfile[6] = { 1, 1, 1, 1, 1, 1 };   // and depths
		FBodyMould Mould;                               // and the front of the torso (FBodyRig::Mould)
		static constexpr float ArmDropDeg = 40.f;

		static FRestSkeleton From(const FBodyRig& B);
		static V3 ArmDir(float Side);
		/** Body (not garment) half-width a and half-depth d at height y: the generic model,
		 * reshaped by the measured width and depth profiles. */
		void TorsoRadii(float y, float& a, float& d) const;
		/**
		 * The torso's section at height y, as two half ovals of half-width a about a centre c
		 * forward of the joints' plane: x = a s, z = c + dFront sqrt(1 - s^2) in front and
		 * z = c - dBack sqrt(1 - s^2) behind. dFront is given for the s = |x| / a asked for: with a
		 * mould of the front (FBodyMould) it follows the measured surface, so a belly stands out in
		 * front of the body and not all around it. The back is not seen and is the generic one.
		 * Without a mould: c = 0 and dFront = dBack = TorsoRadii's d.
		 */
		void TorsoSection(float y, float s, float& a, float& c, float& dFront, float& dBack) const;
		/** Generic torso half-width / half-depth at t = height / torso length (average build). */
		static void BodyModel(float t, float HipW, float SW, float U, float& a, float& d);
	};

	struct FBoneWeights
	{
		int32 Bone[4] = { 0, 0, 0, 0 };    // 0 pelvis, 1 chest, 2 L upper, 3 R upper, 4 L fore, 5 R fore
		float W[4] = { 1, 0, 0, 0 };
	};

	/** Generated garment: one welded, seamless surface plus skinning, anchoring and UV layout. */
	struct FGarmentMeshData
	{
		TArray<V3> Rest, RestN;
		TArray<int32> Tris;
		TArray<FBoneWeights> Weights;
		TArray<float> Looseness;          // 0 = pinned at the shoulders/collar, 1 = free at the hem/cuffs
		TArray<float> Band;               // 1 = an elastic band that holds on to the body (ribbed hem, cuffs)

		// rendering: seams of the UV layout split vertices
		TArray<int32> RenderToSim;        // render vertex -> simulated vertex
		TArray<V2> UV;                    // per render vertex, metres along the fabric
		TArray<int32> RenderTris;

		// ribbed bands (collar, hem, cuffs): cloth triangles clipped to each band, drawn just above
		// the cloth. A band vertex lies on a cloth edge: Lerp(sim A, sim B, T).
		struct FRibVert { int32 A, B; float T; };
		TArray<FRibVert> RibVerts;
		TArray<V2> RibUV;                 // u around the opening, v along it (m): the ribs run along v
		TArray<int32> RibTris;
	};

	/**
	 * Builds a garment as the zero set of a signed distance field (torso, shoulders and sleeves
	 * blended with smooth unions, so armpits and shoulders are one continuous surface), meshed with
	 * surface nets, then cut open at the neck, hem and cuffs. The torso follows the measured build;
	 * below the chest the cut hangs from whatever is widest above it by FGarmentSpec::Fit, and
	 * ribbed hem bands pull back in to the hips. Thread-safe (no engine objects).
	 */
	namespace GarmentMesher
	{
		constexpr float DefaultCell = 0.0125f;
		TSharedPtr<FGarmentMeshData> Build(const FRestSkeleton& S, const FGarmentSpec& G, float Cell = DefaultCell);
		void Normals(const TArray<V3>& V, const TArray<int32>& Tris, TArray<V3>& Out);
	}
}
