#include "SReflectARHud.h"
#include "ReflectARMirror.h"
#include "GarmentSpec.h"
#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "Fonts/FontMeasure.h"
#include "Fonts/SlateFontInfo.h"
#include "Framework/Application/SlateApplication.h"
#include "Rendering/DrawElements.h"
#include "Rendering/SlateRenderer.h"
#include "Styling/CoreStyle.h"
#include "Styling/SlateBrush.h"
#include "Types/SlateEnums.h"

using namespace ReflectAR;

namespace
{
	const FLinearColor Ink(1.f, 1.f, 1.f, 1.f);
	const FLinearColor Accent(1.f, 0.62f, 0.27f, 1.f);                 // warm champagne (linear); the only colour
	const float HoldSeconds = 5.f;                                     // perception's fist-hold time
	const float DragReach = 0.6f;                                      // how far a full swipe carries the strip, in cards
	const float NextChevron = -1.f;                                    // "next" slides the strip left, so the left chevron lights

	FLinearColor WithA(const FLinearColor& C, float A) { return FLinearColor(C.R, C.G, C.B, C.A * A); }
	float HudApproach(float V, float Target, float Rate, float Dt) { return FMath::Lerp(V, Target, 1.f - FMath::Exp(-Rate * Dt)); }
	float HudEaseOut(float T) { T = 1.f - FMath::Clamp(T, 0.f, 1.f); return 1.f - T * T * T; }
}

/** One frame's drawing context. Positions and sizes are in the widget's local units; S converts design pixels. */
struct FReflectHudPainter
{
	FSlateWindowElementList& Out;
	const FGeometry& Geo;
	TSharedRef<FSlateFontMeasure> Measure;
	int32 Layer;
	float S;                 // local units per design pixel
	float Px;                // screen pixels per design pixel
	float Shadow = 0.f;      // text shadow strength (CameraFeed)

	const AReflectARMirror* M = nullptr;
	TSharedPtr<FTrackingFrame> F;
	float W = 0, H = 0, Dt = 0, Feed = 0;
	double Now = 0;
	bool bConnected = false, bTracked = false, bInRange = false;
	float TopUsed = 0.f;     // design pixels taken at the top by the prompt / hint; the toast sits below

	FReflectHudPainter(FSlateWindowElementList& InOut, const FGeometry& InGeo, const TSharedRef<FSlateFontMeasure>& InMeasure, int32 InLayer, float InS)
		: Out(InOut), Geo(InGeo), Measure(InMeasure), Layer(InLayer), S(InS), Px(InS * InGeo.Scale) {}

	FSlateFontInfo Font(float Size, const TCHAR* Face, int32 Tracking) const
	{
		// points at 96 dpi; quantised so a resize doesn't fill the glyph cache
		FSlateFontInfo Info = FCoreStyle::GetDefaultFontStyle(FName(Face), FMath::Max(FMath::RoundToFloat(Size * S * 0.75f * 4.f) / 4.f, 4.f));
		Info.LetterSpacing = Tracking;
		return Info;
	}

	float TextW(const FString& Str, float Size, const TCHAR* Face, int32 Tracking = 0) const
	{
		const FVector2f Sz(Measure->Measure(FStringView(Str), Font(Size, Face, Tracking)));
		return Sz.X;
	}

	/** Text centred vertically on At.Y; AlignX 0 = left edge at At.X, 0.5 = centred, 1 = right. Returns its width. */
	float Text(const FString& Str, const FVector2f& At, float Size, const TCHAR* Face, const FLinearColor& C, float AlignX = 0.5f, int32 Tracking = 0, bool bSnap = true)
	{
		if (Str.IsEmpty()) return 0.f;
		const FSlateFontInfo Info = Font(Size, Face, Tracking);
		const FVector2f Sz(Measure->Measure(FStringView(Str), Info));
		if (C.A < 0.004f || Sz.X <= 0.f || Sz.Y <= 0.f) return Sz.X;
		const FVector2f At0(At.X - Sz.X * AlignX, At.Y - Sz.Y * 0.5f);
		const ESlateDrawEffect Fx = bSnap ? ESlateDrawEffect::None : ESlateDrawEffect::NoPixelSnapping;
		if (Shadow > 0.01f)
			FSlateDrawElement::MakeText(Out, ++Layer, Geo.ToPaintGeometry(Sz, FSlateLayoutTransform(At0 + FVector2f(0.f, 1.5f * S))), Str, Info, Fx, FLinearColor(0, 0, 0, 0.65f * Shadow * C.A));
		FSlateDrawElement::MakeText(Out, ++Layer, Geo.ToPaintGeometry(Sz, FSlateLayoutTransform(At0)), Str, Info, Fx, C);
		return Sz.X;
	}

	/** Rounded box: fill (tinting Tex if given) and an outline of LineW design pixels drawn inside its edge. */
	void Box(const FVector2f& At, const FVector2f& Size, float Radius, const FLinearColor& Fill, const FLinearColor& Line = FLinearColor::Transparent, float LineW = 0.f, UObject* Tex = nullptr)
	{
		if (Size.X <= 0.f || Size.Y <= 0.f || (Fill.A < 0.004f && (Line.A < 0.004f || LineW <= 0.f))) return;
		FSlateBrush Brush;                                             // the element copies what it needs
		Brush.DrawAs = ESlateBrushDrawType::RoundedBox;
		Brush.OutlineSettings = FSlateBrushOutlineSettings(Radius, FSlateColor(Line), LineW > 0.f ? FMath::Max(LineW * S, 1.f / Geo.Scale) : 0.f);
		if (Tex) Brush.SetResourceObject(Tex);
		FSlateDrawElement::MakeBox(Out, ++Layer, Geo.ToPaintGeometry(Size, FSlateLayoutTransform(At)), &Brush, ESlateDrawEffect::None, Fill);
	}

	void Disc(const FVector2f& C, float R, const FLinearColor& Fill) { Box(C - FVector2f(R, R), FVector2f(2 * R, 2 * R), R, Fill); }
	/** Circle stroke centred on radius R. */
	void Ring(const FVector2f& C, float R, const FLinearColor& Col, float Th)
	{
		const float Ro = R + 0.5f * Th * S;
		Box(C - FVector2f(Ro, Ro), FVector2f(2 * Ro, 2 * Ro), Ro, FLinearColor::Transparent, Col, Th);
	}

	/** Antialiased polyline, Th design pixels thick. */
	void Line(TArray<FVector2f> Pts, const FLinearColor& Col, float Th)
	{
		if (Pts.Num() < 2 || Col.A < 0.004f) return;
		FSlateDrawElement::MakeLines(Out, ++Layer, Geo.ToPaintGeometry(), MoveTemp(Pts), ESlateDrawEffect::None, Col, true, FMath::Max(Th * Px, 1.f));
	}

	/** Arc from angle A0 to A1 (radians, 0 = +x, clockwise on screen). */
	void Arc(const FVector2f& C, float R, float A0, float A1, const FLinearColor& Col, float Th)
	{
		const int32 N = FMath::Clamp(FMath::CeilToInt(FMath::Abs(A1 - A0) * R * Geo.Scale / 4.f), 2, 180);
		TArray<FVector2f> Pts;
		Pts.Reserve(N + 1);
		for (int32 k = 0; k <= N; k++)
		{
			const float A = FMath::Lerp(A0, A1, float(k) / N);
			Pts.Add(C + FVector2f(FMath::Cos(A), FMath::Sin(A)) * R);
		}
		Line(MoveTemp(Pts), Col, Th);
	}

	/** "<" (Dir -1) or ">" (Dir +1), centred on C. */
	void Chevron(const FVector2f& C, float Dir, float HalfW, float HalfH, const FLinearColor& Col, float Th)
	{
		Line({ C + FVector2f(-Dir * HalfW, -HalfH), C + FVector2f(Dir * HalfW, 0.f), C + FVector2f(-Dir * HalfW, HalfH) }, Col, Th);
	}

	/** A tick fitting a circle of radius R, drawn up to Progress (0..1). */
	void Tick(const FVector2f& C, float R, const FLinearColor& Col, float Th, float Progress = 1.f)
	{
		const FVector2f A = C + FVector2f(-0.42f, 0.02f) * R, B = C + FVector2f(-0.12f, 0.32f) * R, E = C + FVector2f(0.46f, -0.3f) * R;
		if (Progress <= 0.f) return;
		if (Progress < 0.35f) Line({ A, FMath::Lerp(A, B, Progress / 0.35f) }, Col, Th);
		else Line({ A, B, FMath::Lerp(B, E, FMath::Min((Progress - 0.35f) / 0.65f, 1.f)) }, Col, Th);
	}

	/** Black fading in (down, or up if bUp) over a band of the screen: backing for text over the camera image. */
	void Scrim(float Y, float Height, float A, bool bUp)
	{
		if (A < 0.004f || Height <= 0.f) return;
		TArray<FSlateGradientStop> Stops;
		Stops.Add(FSlateGradientStop(FVector2f(0.f, 0.f), FLinearColor(0, 0, 0, bUp ? A : 0.f)));
		Stops.Add(FSlateGradientStop(FVector2f(W, Height), FLinearColor(0, 0, 0, bUp ? 0.f : A)));
		FSlateDrawElement::MakeGradient(Out, ++Layer, Geo.ToPaintGeometry(FVector2f(W, Height), FSlateLayoutTransform(FVector2f(0.f, Y))), MoveTemp(Stops), Orient_Horizontal);
	}
};

void SReflectARHud::Construct(const FArguments& InArgs, UWorld* InWorld)
{
	World = InWorld;
}

int32 SReflectARHud::OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect, FSlateWindowElementList& OutDrawElements,
	int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const
{
	const UWorld* Wd = World.Get();
	if (!Wd || !FSlateApplication::IsInitialized()) return LayerId;
	if (!Mirror.IsValid()) Mirror = AReflectARMirror::Get(Wd);
	const AReflectARMirror* M = Mirror.Get();
	FSlateRenderer* Renderer = FSlateApplication::Get().GetRenderer();
	if (!M || !M->Client() || !Renderer) return LayerId;
	const FVector2f Size(AllottedGeometry.GetLocalSize());
	const float Gs = AllottedGeometry.Scale;
	if (Size.X < 16.f || Size.Y < 16.f || Gs <= 0.f) return LayerId;

	// scale with the screen as Unity's CanvasScaler (match 0.5) would, whatever the viewport's DPI scale
	const float S = FMath::Sqrt((Size.X * Gs / 1920.f) * (Size.Y * Gs / 1080.f)) / Gs;
	FReflectHudPainter P(OutDrawElements, AllottedGeometry, Renderer->GetFontMeasureService(), LayerId, S);
	const FTrackingClient& C = *M->Client();
	P.M = M;
	P.F = C.Latest;
	P.W = Size.X; P.H = Size.Y;
	P.Now = FTrackingClient::Now();
	P.Dt = St.LastT < 0 ? 0.f : FMath::Clamp(float(P.Now - St.LastT), 0.f, 0.1f);
	St.LastT = P.Now;
	P.bConnected = C.IsConnected();
	P.bTracked = P.bConnected && P.F.IsValid() && P.F->bTracked && P.F->Range != TEXT("none");
	P.bInRange = P.bTracked && P.F->Range == TEXT("ok");
	St.Feed = HudApproach(St.Feed, M->Mode == EMirrorViewMode::CameraFeed ? 1.f : 0.f, 8.f, P.Dt);
	P.Feed = P.Shadow = St.Feed;

	// a selection just landed: confirm at the hand if the ring was up, and always on the carousel
	if (!St.bInit) { St.bInit = true; St.ToastSeen = M->ToastTime; }
	if (M->ToastTime != St.ToastSeen)
	{
		St.ToastSeen = M->ToastTime;
		if (M->ToastText.StartsWith(TEXT("Selected")))
		{
			St.ConfirmT = P.Now;
			St.bConfirmHand = P.Now - St.RingT < 0.6;
			St.RingA = 0.f;
		}
	}

	P.Scrim(0.f, 250.f * S, 0.42f * P.Feed, true);
	P.Scrim(P.H - 360.f * S, 360.f * S, 0.6f * P.Feed, false);
	PaintCarousel(P);
	PaintPrompt(P);
	PaintHint(P);
	PaintFist(P);
	PaintToast(P);
	if (M->bShowDebug) PaintDebug(P);
	return P.Layer;
}

// ======================================================================== carousel
void SReflectARHud::PaintCarousel(FReflectHudPainter& P) const
{
	const TArray<FGarmentSpec>& Specs = GarmentCatalog();
	const int32 N = Specs.Num();
	if (N <= 0) return;
	const AReflectARMirror* M = P.M;
	const float S = P.S, W = P.W, H = P.H;
	const int32 Cur = FMath::Clamp(M->Current, 0, N - 1);
	if (St.Count != N)
	{
		St.Count = N; St.Prev = Cur; St.Base = St.Pos = Cur; St.Vel = 0; St.bLatch = false; St.BrowseSeen = M->LastBrowseTime;
	}
	if (Cur != St.Prev)
	{
		// the catalogue is a ring: take the direction the mirror reports, else the short way round
		int32 Step = Cur - St.Prev;
		if (Step > N / 2) Step -= N; else if (Step < -(N / 2)) Step += N;
		const int32 Dir = M->LastBrowseDir;
		if (M->LastBrowseTime != St.BrowseSeen && Dir != 0 && (Step - Dir) % N == 0) Step = Dir;
		St.Base += Step; St.Prev = Cur;
		St.StepDir = Step > 0 ? 1 : -1; St.StepT = P.Now;
		St.bLatch = true;
		if (P.bInRange) St.Browses++;
	}
	St.BrowseSeen = M->LastBrowseTime;

	// follow the hand; once a swipe has completed its drag is spent (Base has stepped instead)
	const float Drag = FMath::Clamp(M->BrowseDrag, -1.f, 1.f);
	if (St.bLatch && (FMath::Abs(Drag) < 0.05f || Drag * St.StepDir < 0.f)) St.bLatch = false;
	const float Eff = (St.bLatch || N < 2) ? 0.f : Drag;
	{
		// critically damped spring: keeps its speed through the hand-over from drag to step, and springs back if abandoned
		const double Target = St.Base + double(Eff) * DragReach, Om = 20.0;
		const double D = St.Pos - Target, Tmp = (St.Vel + Om * D) * P.Dt, E = FMath::Exp(-Om * P.Dt);
		St.Pos = Target + (D + Tmp) * E;
		St.Vel = (St.Vel - Om * Tmp) * E;
	}
	St.CarA = HudApproach(St.CarA, P.bInRange ? 1.f : (P.bConnected ? 0.55f : 0.3f), 5.f, P.Dt);
	const float CarA = St.CarA;

	// signed distance of card i from the focus, in cards, the short way round the ring
	auto Off = [this, N](int32 i) { double d = i - St.Pos; d -= N * FMath::RoundToDouble(d / N); return float(d); };
	const FVector2f Focus(W * 0.5f, H - 120.f * S);
	const float RingR = 62.f * S;

	for (int32 i = 0; i < N; i++)
	{
		const float d = Off(i), a = FMath::Abs(d);
		const float X = FMath::Sign(d) * (a <= 1.f ? a * 136.f : 136.f + (a - 1.f) * 100.f) * S;
		const float Dia = (a <= 1.f ? FMath::Lerp(104.f, 66.f, FMath::SmoothStep(0.f, 1.f, a)) : FMath::Lerp(66.f, 54.f, FMath::Min(a - 1.f, 1.f))) * S;
		float A = a <= 1.f ? FMath::Lerp(1.f, 0.55f, a) : (a <= 2.f ? FMath::Lerp(0.55f, 0.25f, a - 1.f) : FMath::Lerp(0.25f, 0.f, FMath::Min(a - 2.f, 1.f)));
		if (N > 1) A *= FMath::Clamp((N * 0.5f - a) * 2.f, 0.f, 1.f);                           // fade out where the ring wraps
		A *= FMath::Clamp((W * 0.5f - 96.f * S - FMath::Abs(X)) / (40.f * S), 0.f, 1.f);         // and before the chevrons
		A *= CarA;
		if (A < 0.01f) continue;
		const FVector2f Cc = Focus + FVector2f(X, 0.f);
		const float R = Dia * 0.5f;
		// swatch: a piece of the actual fabric, with a rim so dark fabrics still show on the mirror
		if (UTexture2D* Tex = M->Swatch(i)) P.Box(Cc - FVector2f(R, R), FVector2f(Dia, Dia), R, WithA(Ink, A), WithA(Ink, 0.45f * A), 1.5f, Tex);
		else P.Box(Cc - FVector2f(R, R), FVector2f(Dia, Dia), R, WithA(Specs[i].ColorA, A), WithA(Ink, 0.45f * A), 1.5f);
		if (i == M->Selected)
		{
			const float Br = 0.24f * R + 2.f * S;
			const FVector2f Bc = Cc + FVector2f(0.72f, -0.72f) * R;
			P.Disc(Bc, Br, WithA(Accent, A));
			P.Tick(Bc, Br, FLinearColor(0, 0, 0, 0.9f * A), 2.f);
		}
	}

	// the focus: a fixed lens the swatches slide through, and the name of whichever is nearest
	const int32 J = ((int32(FMath::RoundToDouble(St.Pos)) % N) + N) % N;
	const float dJ = Off(J), Centred = FMath::Clamp(1.f - FMath::Abs(dJ) * 2.2f, 0.f, 1.f);
	const bool bSel = J == M->Selected;
	P.Ring(Focus, RingR, WithA(bSel ? FMath::Lerp(Ink, Accent, Centred) : Ink, 0.9f * CarA), 2.f);
	const float Ct = float(P.Now - St.ConfirmT) / 0.7f;
	if (Ct >= 0.f && Ct < 1.f) P.Ring(Focus, RingR + 30.f * S * HudEaseOut(Ct), WithA(Accent, FMath::Square(1.f - Ct)), 2.f);

	const FGarmentSpec& Sp = Specs[J];
	const float Nx = Focus.X + dJ * 48.f * S, Na = Centred * CarA;
	const bool bDetail = !Sp.Detail.IsEmpty();
	if (bSel) P.Text(TEXT("SELECTED"), FVector2f(Nx, H - (bDetail ? 288.f : 270.f) * S), 15.f, TEXT("Medium"), WithA(Accent, Na), 0.5f, 220, false);
	P.Text(Sp.Name, FVector2f(Nx, H - (bDetail ? 244.f : 226.f) * S), 38.f, TEXT("Light"), WithA(Ink, Na), 0.5f, 0, false);
	if (bDetail) P.Text(Sp.Detail, FVector2f(Nx, H - 204.f * S), 21.f, TEXT("Regular"), WithA(Ink, 0.62f * Na), 0.5f, 0, false);

	// position
	if (N > 1 && N <= 8)
	{
		for (int32 i = 0; i < N; i++)
		{
			const float n = FMath::Clamp(1.f - FMath::Abs(Off(i)), 0.f, 1.f);
			const float Dw = (6.f + 16.f * n) * S, Dh = 6.f * S;
			P.Box(FVector2f(W * 0.5f + (i - (N - 1) * 0.5f) * 22.f * S - Dw * 0.5f, H - 34.f * S - Dh * 0.5f), FVector2f(Dw, Dh), Dh * 0.5f, WithA(Ink, (0.3f + 0.7f * n) * CarA));
		}
	}
	else if (N > 8) P.Text(FString::Printf(TEXT("%d / %d"), J + 1, N), FVector2f(W * 0.5f, H - 34.f * S), 17.f, TEXT("Medium"), WithA(Ink, 0.7f * CarA), 0.5f, 100);

	// chevrons: the one the strip is being dragged toward lights, leans out and trails echoes
	if (N > 1)
	{
		St.Chev = HudApproach(St.Chev, Eff, 18.f, P.Dt);
		const float Pulse = FMath::Clamp(1.f - float(P.Now - St.StepT) / 0.45f, 0.f, 1.f) * St.StepDir;
		const float V = FMath::Clamp(St.Chev + Pulse, -1.f, 1.f);
		const float ChevX = FMath::Min(W * 0.5f - 44.f * S, ((N <= 3 ? 169.f : 263.f) + 58.f) * S);
		for (int32 k = 0; k < 2; k++)
		{
			const float Side = k == 0 ? -1.f : 1.f;
			const float Act = FMath::Max(0.f, V * Side * NextChevron), Other = FMath::Max(0.f, -V * Side * NextChevron);
			const FVector2f Cc(Focus.X + Side * (ChevX + Act * 12.f * S), Focus.Y);
			const FLinearColor Col = WithA(FMath::Lerp(WithA(Ink, 0.4f), Accent, Act), CarA * (1.f - 0.6f * Other));
			P.Chevron(Cc, Side, 7.f * S, 14.f * S, Col, 2.5f + Act);
			for (int32 e = 1; e <= 2; e++)
				P.Chevron(Cc - FVector2f(Side * e * 13.f * S, 0.f), Side, 7.f * S, 14.f * S, WithA(Accent, FMath::Clamp(Act * 3.f - e, 0.f, 1.f) * (1.f - 0.3f * e) * CarA), 2.5f);
		}
	}
}

// ======================================================================== range prompt
void SReflectARHud::PaintPrompt(FReflectHudPainter& P) const
{
	const float S = P.S, W = P.W;
	const TSharedPtr<FTrackingFrame>& F = P.F;
	int32 Kind = 0;
	if (!P.bConnected)
	{
		Kind = 1; St.PromptTitle = TEXT("Starting…");
		St.PromptSub = P.M->bShowDebug ? TEXT("Waiting for the perception service (perception/server.py)") : TEXT("Waiting for the perception service");
	}
	else if (!F.IsValid() || !F->bTracked || F->Range == TEXT("none")) { Kind = 2; St.PromptTitle = TEXT("Step into view"); St.PromptSub = TEXT("Stand 1–3 m from the mirror, facing it"); }
	else if (F->Range == TEXT("too_close")) { Kind = 3; St.PromptTitle = TEXT("Please step back"); }
	else if (F->Range == TEXT("too_far")) { Kind = 4; St.PromptTitle = TEXT("Come a little closer"); }
	if (Kind >= 3)
	{
		St.PromptSub = FString::Printf(TEXT("You’re %.1f m away"), F->Dist);
		St.PromptDist = St.PromptKind >= 3 ? HudApproach(St.PromptDist, F->Dist, 8.f, P.Dt) : F->Dist;
	}
	if (Kind) St.PromptKind = Kind;
	St.PromptA = HudApproach(St.PromptA, Kind ? 1.f : 0.f, 7.f, P.Dt);
	const float A = St.PromptA;
	if (A < 0.01f) return;
	P.TopUsed = FMath::Max(P.TopUsed, A * (St.PromptKind >= 2 ? 205.f : 140.f));

	const float Yo = (A - 1.f) * 10.f * S, Cx = W * 0.5f;
	P.Box(FVector2f(Cx - 18.f * S, 50.f * S + Yo), FVector2f(36.f * S, 2.f * S), 1.f * S, WithA(Accent, A));
	P.Text(St.PromptTitle, FVector2f(Cx, 100.f * S + Yo), 56.f, TEXT("Light"), WithA(Ink, A));
	P.Text(St.PromptSub, FVector2f(Cx, 152.f * S + Yo), 23.f, TEXT("Regular"), WithA(Ink, 0.72f * A));
	if (St.PromptKind < 2) return;

	// where to stand: 0..4 m with the 1-3 m band marked, and the user on it
	const float Gw = 300.f * S, Gy = 200.f * S + Yo, X0 = Cx - Gw * 0.5f;
	auto At = [X0, Gw](float Metres) { return X0 + FMath::Clamp(Metres, 0.f, 4.f) / 4.f * Gw; };
	P.Line({ FVector2f(X0, Gy), FVector2f(X0 + Gw, Gy) }, WithA(Ink, 0.28f * A), 1.5f);
	P.Line({ FVector2f(At(1.f), Gy), FVector2f(At(3.f), Gy) }, WithA(Ink, 0.95f * A), 3.f);
	for (float Mark : { 1.f, 3.f })
	{
		P.Line({ FVector2f(At(Mark), Gy - 6.f * S), FVector2f(At(Mark), Gy + 6.f * S) }, WithA(Ink, 0.95f * A), 2.f);
		P.Text(FString::Printf(TEXT("%.0f m"), Mark), FVector2f(At(Mark), Gy + 24.f * S), 16.f, TEXT("Regular"), WithA(Ink, 0.6f * A));
	}
	if (St.PromptKind >= 3) P.Disc(FVector2f(At(St.PromptDist), Gy), 7.f * S, WithA(Accent, A));
}

// ======================================================================== gesture hint
void SReflectARHud::PaintHint(FReflectHudPainter& P) const
{
	// shown until the user has browsed a couple of times; comes back once nobody has been there for a while
	if (P.bTracked) St.TrackedT = P.Now;
	else if (P.Now - St.TrackedT > 8.0) St.Browses = 0;
	St.HintA = HudApproach(St.HintA, P.bInRange && St.Browses < 2 ? 1.f : 0.f, 4.f, P.Dt);
	const float A = St.HintA * (1.f - St.PromptA);
	if (A < 0.01f) return;

	const float S = P.S, W = P.W;
	const TCHAR* Labels[2] = { TEXT("SWIPE TO BROWSE"), TEXT("HOLD A FIST TO SELECT") };
	const float Icon = 60.f * S, Gap = 14.f * S, Between = 64.f * S;
	const float Tw[2] = { P.TextW(Labels[0], 17.f, TEXT("Medium"), 160), P.TextW(Labels[1], 17.f, TEXT("Medium"), 160) };
	const float Gw[2] = { Icon + Gap + Tw[0], Icon + Gap + Tw[1] };
	const bool bStack = Gw[0] + Between + Gw[1] > W - 64.f * S;
	P.TopUsed = FMath::Max(P.TopUsed, A * (bStack ? 150.f : 80.f));
	const FLinearColor Col = WithA(Ink, 0.85f * A);

	for (int32 g = 0; g < 2; g++)
	{
		const float X = bStack ? (W - Gw[g]) * 0.5f : (W - Gw[0] - Between - Gw[1]) * 0.5f + (g ? Gw[0] + Between : 0.f);
		const float Y = (bStack ? 66.f + 68.f * g : 76.f) * S;
		const FVector2f C(X + Icon * 0.5f, Y);
		auto V = [&C, S](float Lx, float Ly) { return C + FVector2f(Lx, Ly) * S; };
		if (g == 0)
		{
			// an open hand gliding side to side between two chevrons
			const float Ox = FMath::Sin(float(FMath::Fmod(P.Now, 2.6) / 2.6) * 2.f * PI) * 6.f;
			P.Chevron(V(-29.f, 0.f), -1.f, 3.5f * S, 7.f * S, WithA(Col, 0.6f), 2.f);
			P.Chevron(V(29.f, 0.f), 1.f, 3.5f * S, 7.f * S, WithA(Col, 0.6f), 2.f);
			P.Box(V(Ox - 10.f, 0.f), FVector2f(20.f, 16.f) * S, 6.f * S, FLinearColor::Transparent, Col, 2.f);
			const float Tops[4] = { -11.f, -15.f, -13.5f, -9.f };
			for (int32 f = 0; f < 4; f++) P.Line({ V(Ox - 7.5f + 5.f * f, 1.f), V(Ox - 7.5f + 5.f * f, Tops[f]) }, Col, 2.f);
			P.Line({ V(Ox - 9.5f, 9.f), V(Ox - 16.f, 2.f) }, Col, 2.f);
		}
		else
		{
			// a closed fist inside a ring that fills, as the real one does at the hand
			const float Ph = float(FMath::Fmod(P.Now, 3.2));
			P.Ring(C, 25.f * S, WithA(Col, 0.3f), 1.5f);
			P.Arc(C, 25.f * S, -HALF_PI, -HALF_PI + 2.f * PI * FMath::Clamp(Ph / 2.4f, 0.f, 1.f), WithA(Accent, A * FMath::Clamp((3.1f - Ph) / 0.4f, 0.f, 1.f)), 3.f);
			P.Box(V(-10.f, -8.f), FVector2f(20.f, 17.f) * S, 6.f * S, FLinearColor::Transparent, Col, 2.f);
			for (int32 f = -1; f <= 1; f++) P.Line({ V(5.f * f, -7.f), V(5.f * f, -1.f) }, Col, 2.f);
			P.Line({ V(-8.f, 3.f), V(4.f, 3.f) }, Col, 2.f);
		}
		P.Text(Labels[g], FVector2f(X + Icon + Gap, Y), 17.f, TEXT("Medium"), Col, 0.f, 160);
	}
}

// ======================================================================== fist ring at the hand
void SReflectARHud::PaintFist(FReflectHudPainter& P) const
{
	const AReflectARMirror* M = P.M;
	const TSharedPtr<FTrackingFrame>& F = P.F;
	const float S = P.S, W = P.W, H = P.H, R = 54.f * S;
	const float Ct = float(P.Now - St.ConfirmT) / 0.9f;
	const bool bConfirming = St.bConfirmHand && Ct >= 0.f && Ct < 1.f;

	const float Raw = F.IsValid() && F->bTracked ? F->FistProgress : 0.f;
	bool bOn = false;
	if (Raw > 0.01f && M->Body().Valid && !bConfirming)
	{
		V3 Wp = M->Body().J[F->FistHand == TEXT("left") ? LM::LWrist : LM::RWrist];
		if (M->Mode == EMirrorViewMode::Mirror) Wp.Z = -Wp.Z;          // the reflected hand
		FVector2D Sp;
		if (M->BodyToScreen(Wp, FVector2D(W, H), Sp))
		{
			// keep it on screen and clear of the carousel
			St.RingPos = FVector2f(FMath::Clamp(float(Sp.X), 84.f * S, W - 84.f * S), FMath::Clamp(float(Sp.Y), 84.f * S, FMath::Max(H - 380.f * S, 84.f * S)));
			St.RingT = P.Now;
			bOn = true;
		}
	}
	// perception steps at its own rate: ease the fill, but never lag a reset
	if (bOn) St.Prog = (Raw > St.Prog && St.RingA > 0.05f) ? HudApproach(St.Prog, Raw, 25.f, P.Dt) : Raw;
	St.RingA = HudApproach(St.RingA, bOn ? 1.f : 0.f, bOn ? 16.f : 9.f, P.Dt);

	const FVector2f C = St.RingPos;
	const float A = St.RingA;
	if (A > 0.01f)
	{
		const float Prog = FMath::Clamp(St.Prog, 0.f, 1.f), End = -HALF_PI + 2.f * PI * Prog;   // clockwise from the top
		P.Disc(C, R + 12.f * S, FLinearColor(0, 0, 0, 0.3f * P.Feed * A));
		P.Ring(C, R, WithA(Ink, 0.22f * A), 2.f);
		P.Arc(C, R, -HALF_PI, End, WithA(Accent, A), 6.f);
		P.Disc(C + FVector2f(FMath::Cos(End), FMath::Sin(End)) * R, 5.f * S, WithA(Accent, A));
		P.Text(FString::FromInt(FMath::Max(1, FMath::CeilToInt((1.f - Prog) * HoldSeconds))), C, 40.f, TEXT("Light"), WithA(Ink, A), 0.5f, 0, false);
	}
	if (bConfirming)
	{
		// the ring closes, swells and fades around a tick
		const float Fa = FMath::Pow(1.f - Ct, 1.5f);
		P.Arc(C, R * (1.f + 0.28f * HudEaseOut(Ct)), 0.f, 2.f * PI, WithA(Accent, Fa), 6.f * (1.f - 0.5f * Ct));
		P.Tick(C, R, WithA(Ink, FMath::Clamp((1.f - Ct) / 0.35f, 0.f, 1.f)), 5.f, FMath::Clamp(Ct / 0.28f, 0.f, 1.f));
	}
}

// ======================================================================== toast
void SReflectARHud::PaintToast(FReflectHudPainter& P) const
{
	const AReflectARMirror* M = P.M;
	const float Age = float(P.Now - M->ToastTime);
	const float A = FMath::Clamp(Age / 0.15f, 0.f, 1.f) * FMath::Clamp(2.5f - Age, 0.f, 1.f);
	if (A < 0.01f || M->ToastText.IsEmpty()) return;
	const float S = P.S, W = P.W;
	const bool bTick = M->ToastText.StartsWith(TEXT("Selected"));
	const float Lead = bTick ? 30.f * S : 0.f, Pad = 26.f * S, Avail = W - 96.f * S - Lead - 2.f * Pad;
	float Size = 23.f, Tw = P.TextW(M->ToastText, Size, TEXT("Regular"));
	if (Tw > Avail && Tw > 0.f) { Size = FMath::Max(Size * Avail / Tw, 9.f); Tw = P.TextW(M->ToastText, Size, TEXT("Regular")); }

	// a pill under whatever else is at the top
	const float Ph = 50.f * S, Pw = Tw + Lead + 2.f * Pad;
	const FVector2f At((W - Pw) * 0.5f, (38.f + P.TopUsed) * S - (1.f - HudEaseOut(Age / 0.25f)) * 8.f * S);
	P.Box(At, FVector2f(Pw, Ph), Ph * 0.5f, FLinearColor(0, 0, 0, 0.5f * P.Feed * A), WithA(Ink, 0.4f * A), 1.5f);
	if (bTick) P.Tick(At + FVector2f(Pad + 9.f * S, Ph * 0.5f), 16.f * S, WithA(Accent, A), 2.5f);
	P.Text(M->ToastText, FVector2f(At.X + Pad + Lead, At.Y + Ph * 0.5f), Size, TEXT("Regular"), WithA(Ink, A), 0.f);
}

// ======================================================================== debug lines (developers)
void SReflectARHud::PaintDebug(FReflectHudPainter& P) const
{
	const AReflectARMirror* M = P.M;
	const TSharedPtr<FTrackingFrame>& F = P.F;
	const float S = P.S, W = P.W;
	const float Size = FMath::Max(15.f, 11.f / FMath::Max(P.Px, 0.01f));          // small, but never under 11 screen pixels
	const float Margin = 14.f * S, LineH = Size * 1.45f * S, Sep = Size * 1.3f * S;
	const FLinearColor Col = WithA(Ink, 0.62f);

	TArray<FString> Seg;
	if (!F.IsValid())
	{
		Seg.Add(P.bConnected ? TEXT("connected") : TEXT("no perception"));
		Seg.Add(TEXT("[M] mode")); Seg.Add(TEXT("[</>] browse")); Seg.Add(TEXT("[Space] select")); Seg.Add(TEXT("[B] proxy"));
		Seg.Add(TEXT("[C] cube")); Seg.Add(TEXT("[L] latency test")); Seg.Add(TEXT("[D] debug"));
	}
	else
	{
		Seg.Add(M->Mode == EMirrorViewMode::Mirror ? TEXT("Mirror") : TEXT("CameraFeed"));
		Seg.Add(FString::Printf(TEXT("perception %.0f fps  depth %.0f Hz"), F->ProcFps, F->DepthHz));
		Seg.Add(FString::Printf(TEXT("dist %.2f m (skel %.2f / mono %.2f, s=%.2f)"), F->Dist, F->ZSkel, F->ZDa, F->Scale));
		Seg.Add(FString::Printf(TEXT("render %.0f fps"), M->RenderFps));
		Seg.Add(TEXT("[M] mode [B] proxy [C] cube [D] debug"));
	}
	// flow the segments so a narrow or portrait screen wraps instead of clipping
	float X = Margin, Y = 12.f * S + LineH * 0.5f;
	for (int32 i = 0; i < Seg.Num(); i++)
	{
		const float Tw = P.TextW(Seg[i], Size, TEXT("Regular"));
		if (X > Margin && X + Tw > W - Margin) { X = Margin; Y += LineH; }
		else if (X > Margin) P.Text(TEXT("·"), FVector2f(X - Sep * 0.5f, Y), Size, TEXT("Regular"), WithA(Col, 0.6f));
		P.Text(Seg[i], FVector2f(X, Y), Size, TEXT("Regular"), Col, 0.f);
		X += Tw + Sep;
	}
	if (F.IsValid()) P.Text(M->LatencyLine, FVector2f(Margin, Y + LineH), Size, TEXT("Regular"), Col, 0.f);
}
