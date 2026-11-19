#pragma once

#include "CoreMinimal.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Widgets/SLeafWidget.h"

class AReflectARMirror;
class UWorld;
struct FReflectHudPainter;

/**
 * The on-screen UI, painted straight into the game viewport: status prompt and gesture hint (top),
 * garment carousel (bottom edge, off the body), fist-hold ring at the hand, toast, debug lines.
 *
 * Behind a two-way mirror black is the reflection and anything bright glows, so the UI is thin
 * light strokes and light text with one accent colour; CameraFeed mode adds soft scrims and text
 * shadows to stay legible over the camera image. Laid out in 1920 x 1080 design pixels, scaled
 * with the viewport. Reads AReflectARMirror every frame; it owns no game state.
 */
class SReflectARHud : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SReflectARHud) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs, UWorld* InWorld);

	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect, FSlateWindowElementList& OutDrawElements,
		int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override;
	virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(1920.0, 1080.0); }

protected:
	virtual bool ComputeVolatility() const override { return true; }          // animated: repaint every frame

private:
	/** Animation state, advanced once per paint on the wall clock. */
	struct FState
	{
		double LastT = -1;
		bool bInit = false;
		float Feed = 0.f;                        // 0 Mirror .. 1 CameraFeed

		// carousel: Base is Current unwrapped (it keeps counting past the ends), Pos follows Base + the swipe
		int32 Count = 0, Prev = 0, StepDir = 0;
		double Base = 0, Pos = 0, Vel = 0, StepT = -10, BrowseSeen = -10;
		bool bLatch = false;                     // ignore BrowseDrag until it has let go of a completed swipe
		float Chev = 0.f, CarA = 0.f;

		// prompt and hint
		int32 PromptKind = 0;
		FString PromptTitle, PromptSub;
		float PromptA = 0.f, PromptDist = 2.f, HintA = 0.f;
		int32 Browses = 0;
		double TrackedT = -100;

		// fist ring and the selection confirmation
		float Prog = 0.f, RingA = 0.f;
		FVector2f RingPos = FVector2f::ZeroVector;
		double RingT = -10, ToastSeen = -10, ConfirmT = -10;
		bool bConfirmHand = false;
	};

	TWeakObjectPtr<UWorld> World;
	mutable TWeakObjectPtr<AReflectARMirror> Mirror;
	mutable FState St;

	void PaintCarousel(FReflectHudPainter& P) const;
	void PaintPrompt(FReflectHudPainter& P) const;
	void PaintHint(FReflectHudPainter& P) const;
	void PaintFist(FReflectHudPainter& P) const;
	void PaintToast(FReflectHudPainter& P) const;
	void PaintDebug(FReflectHudPainter& P) const;
};
