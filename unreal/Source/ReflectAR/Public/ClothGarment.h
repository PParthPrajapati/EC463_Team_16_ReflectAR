#pragma once

#include "GarmentTrims.h"
#include "GarmentAsset.h"
#include "Async/Future.h"

namespace ReflectAR
{
	/**
	 * A garment simulated as cloth. The seamless mesh from GarmentMesher is skinned to six bones
	 * driven by the tracked skeleton; that skinned shape is the "body-fitted" target. An XPBD
	 * solver then simulates the fabric: inextensible stretch, soft compression (so slack buckles into
	 * folds), bending, gravity, inertia, damping and collisions against the user's body (a torso of
	 * elliptical sections plus limb capsules). The garment is cut with real ease, sewn to the body's
	 * motion only at the yoke, collar and sleeve caps, and hangs free from there: the drape comes
	 * from the cloth resting on the body, with a leash to the skinned target as a safety limit.
	 *
	 * The solver runs on the welded mesh. What is drawn (BodySection) is a copy split along the UV
	 * seams, with normals from the welded mesh (no shading seams) and tangents for the normal map,
	 * plus the front closure (Trims) riding on it. Pure C++: AReflectARMirror uploads the sections.
	 */
	class FClothGarment
	{
	public:
		FClothGarment(const FGarmentSpec& InSpec, const FBodyRig& InBody) : Spec(InSpec), Body(InBody) {}

		const FGarmentSpec& Spec;
		int Substeps = 8;                 // small steps keep the fabric from stretching
		float Follow = 0.85f;              // share of body motion the cloth inherits directly
		float Alpha = 1.f;

		float LastBuildMs = 0, LastStepMs = 0;
		int RebuildCount = 0;
		int VertexCount() const { return X.Num(); }
		bool HasMesh() const { return bHasMesh; }

		FMeshSection BodySection;
		FMeshSection RibSection;          // ribbed collar / hem / cuffs, empty if the garment has none
		TSharedPtr<FGarmentTrims> Trims;

		/** Start (re)generating this garment for the current body if needed. Cheap; call every frame. */
		void Prepare(double Now);
		/** Simulates one frame and refreshes BodySection / Trims. */
		void Step(float Dt, double Now);
		/** True once after a new mesh was swapped in: its sections must be re-created. */
		bool ConsumeTopologyChanged() { const bool b = bTopologyChanged; bTopologyChanged = false; return b; }

		/** How far the cloth may stand off the body (ease plus drape), with a little to spare. */
		float Room() const { return (Spec.Ease + 0.03f * Spec.Drape + 0.01f) * Body.Unit(); }
		/** Where the garment covers the torso: its torso and shoulder colliders grown by Room(). */
		void Outline(TArray<FCapsuleB>& Into) const;

	private:
		struct FBuilt
		{
			FRestSkeleton Skel;
			TSharedPtr<FGarmentMeshData> Data;
			TArray<float> MaxDist, SkinK, ERest, BRest;
			TArray<int32> EA, EB, BA, BB;
			float Ms = 0, MeanEdge = 0.02f;
			bool bAuthored = false;
			TArray<float> EdgeDist, InvDeg;
		};
		static TSharedPtr<FBuilt> Generate(FRestSkeleton Skel, const FGarmentSpec* Spec, TSharedPtr<const FGarmentAsset> Asset);
		void ApplyPending(double Now);
		bool Drifted() const;
		void SkinTargets();
		void BuildColliders();
		bool CollideTorso(V3& P, V3& OutN) const;
		void UpdateRenderMesh();
		void Tangents(FMeshSection& Sec);

		const FBodyRig& Body;
		TSharedPtr<FGarmentMeshData> Data;
		FRestSkeleton Built;
		bool bHasMesh = false, bSimInit = false, bTopologyChanged = false, bAuthored = false;
		TFuture<TSharedPtr<FBuilt>> Pending;
		double DriftSince = -1, LastApplied = -100, FirstApplied = -100, LastStep = -100;

		// simulation state
		TArray<V3> X, Prev, Target, LastTarget;
		TArray<float> MaxDist, SkinK, ERest, BRest;
		TArray<int32> EA, EB, BA, BB;
		FFrame RestFrames[6];
		FAffine SkinM[6];
		TArray<FCapsuleB> Colliders;
		TArray<V3> ColCenter;
		TArray<float> ColBound;
		static constexpr int TorsoRows = 48, FirstLimb = 5;       // Colliders[0..4]: torso levels, for Outline() only
		static constexpr float Thick = 0.004f, Friction = 0.3f;
		static constexpr float MaxSag = 0.012f;                   // how far the cloth may hang below its cut (m)
		// the torso's section per height: half-width, centre forward of the joints, half-depth behind
		// it and in front of it (at FBodyMould's columns: the front follows the mould of the body)
		float TorsoA[TorsoRows] = {}, TorsoC[TorsoRows] = {}, TorsoB[TorsoRows] = {}, TorsoF[TorsoRows][FBodyMould::Cols] = {};
		float TorsoT0 = 0, TorsoT1 = 1, TorsoScale = 1;
		V3 TorsoR[TorsoRows];

		// render mesh (split at UV seams)
		TArray<V3> SimN, TAcc, BAcc;

		// shading data per simulated vertex: 0..1 over EdgeBand from an edge of the cloth; 1 / its
		// number of neighbours; how open it is (creases and the cloth under an arm are darker)
		TArray<float> EdgeDist, InvDeg, Ao;
		TArray<V3> Lap;
		float MeanEdge = 0.02f;
		void Shade();

	public:
		/** Width of the band along every edge of the garment that EdgeDist (vertex colour G) spans, m. */
		static constexpr float EdgeBand = 0.06f;
	};
}
