// Swept queries answer whether two convex polygons, each translating at a
// constant velocity over the normalized time window [0,1], come into contact
// at any instant during the motion. This file contains only the motion data
// model, input validation and orchestration; the conservative-advancement
// iteration lives in sweep.go and deliberately reuses the single-frame
// Evaluate kernel as its only geometric primitive.
package geometry

import "math"

// Sweep status values reported to clients. They are distinct from the static
// Evaluate status strings so the two query kinds never get conflated.
const (
	// StatusContact: the parts touch for the first time at Time within [0,1].
	StatusContact = "contact"
	// StatusSafe: the parts stay strictly separated over the whole window.
	StatusSafe = "safe"
)

// Motion is one part on a constant-velocity line:
//
//	pose(t) = Offset + velocity*t.
//
// Polygon vertices are given in the part's local frame; Offset places the
// local frame at t=0. A zero offset (the common case) leaves the vertices as
// the initial world pose.
type Motion struct {
	Vertices []Vec2
	Offset   Vec2
	Velocity Vec2
}

// SweepResult is the kernel answer for one swept pair.
//
// When Status == StatusContact:
//   - Time is the true time of first impact (a root found by convergence, not
//     a sampled instant), Normal/PointA/PointB are the tangency geometry at
//     that instant, and PenetrationDepth is 0 (first contact is a touch).
//
// When Status == StatusSafe:
//   - Time is the instant of minimum clearance over [0,1], Distance is that
//     minimum gap and Normal is the gap direction at that instant.
type SweepResult struct {
	// Status is StatusContact or StatusSafe.
	Status string `json:"status"`
	// Time is the time of impact (contact) or of closest approach (safe),
	// always in [0,1].
	Time float64 `json:"time"`
	// Distance is the minimum clearance over the window (0 at first contact).
	Distance float64 `json:"distance"`
	// PenetrationDepth is the static depth reported at the contact instant
	// (0 for a genuine first touch; nonzero only when contact is inherited
	// from an initially penetrating pose).
	PenetrationDepth float64 `json:"penetrationDepth"`
	// Normal is the contact/gap normal at Time, pointing from A toward B.
	Normal Vec2 `json:"normal"`
	// PointA / PointB are the contact or closest points (world frame) at Time.
	PointA Vec2 `json:"pointA"`
	PointB Vec2 `json:"pointB"`
	// Iterations is the number of single-frame Evaluate calls consumed.
	Iterations int `json:"iterations"`
}

// Sweep evaluates the swept query for two parts in constant linear motion
// over t in [0,1]. It validates both contours with the same rules as the
// static query (too few vertices, degenerate, non-convex, non-finite
// coordinates) and additionally rejects non-finite velocity or offset
// components. Relative motion is governed solely by velocityA-velocityB.
func Sweep(a, b Motion) (*SweepResult, error) {
	pa, err := NewPolygon(a.Vertices)
	if err != nil {
		return nil, namedError("A", err.(*KernelError))
	}
	pb, err := NewPolygon(b.Vertices)
	if err != nil {
		return nil, namedError("B", err.(*KernelError))
	}
	if !isFinite(a.Offset.X) || !isFinite(a.Offset.Y) {
		return nil, &KernelError{Code: ErrNonFiniteCoordinate,
			Message: "polygon A: initial offset must contain finite coordinates"}
	}
	if !isFinite(b.Offset.X) || !isFinite(b.Offset.Y) {
		return nil, &KernelError{Code: ErrNonFiniteCoordinate,
			Message: "polygon B: initial offset must contain finite coordinates"}
	}
	if !isFinite(a.Velocity.X) || !isFinite(a.Velocity.Y) {
		return nil, &KernelError{Code: ErrNonFiniteVelocity,
			Message: "polygon A: velocity components must be finite numbers"}
	}
	if !isFinite(b.Velocity.X) || !isFinite(b.Velocity.Y) {
		return nil, &KernelError{Code: ErrNonFiniteVelocity,
			Message: "polygon B: velocity components must be finite numbers"}
	}

	s := &sweeper{
		a: pa, b: pb,
		oa: a.Offset, ob: b.Offset,
		va: a.Velocity, vb: b.Velocity,
		cache: map[float64]*Result{},
	}
	return s.run()
}

// contactTol is the absolute clearance at which a separated configuration is
// declared "in contact". Contact is a geometric condition, so it scales only
// with coordinate magnitude (floored at 1), matching the kernel's own epsFor
// scaling — never with velocity.
func sweepContactTol(maxCoord float64) float64 {
	m := math.Max(1.0, maxCoord)
	return contactRelTol * m
}

// contactRelTol is the dimensionless relative contact tolerance: a residual
// gap below ~1e-10 of the coordinate scale is numerical contact (the same
// order as the single-frame kernel's own tolerance).
const contactRelTol = 1e-10
