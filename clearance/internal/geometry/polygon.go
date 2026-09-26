package geometry

import "math"

// Polygon is a planar convex polygon given by its boundary vertices in order
// (clockwise or counter-clockwise both accepted).
type Polygon struct {
	// Vertices are the validated input vertices in their original order.
	Vertices []Vec2
	// CCW is true when the boundary is oriented counter-clockwise.
	CCW bool
	// scale is a characteristic size used to scale tolerances.
	scale float64
}

// NewPolygon validates the input and builds a Polygon.
//
// It rejects: fewer than three vertices, non-finite coordinates, zero-area
// (degenerate) loops and non-convex loops. Consecutive collinear vertices are
// accepted, but a concave polygon is refused outright — no convex
// decomposition is performed.
func NewPolygon(pts []Vec2) (*Polygon, error) {
	if len(pts) < 3 {
		return nil, &KernelError{Code: ErrTooFewVertices,
			Message: "polygon must contain at least 3 vertices"}
	}
	for i, p := range pts {
		if !isFinite(p.X) || !isFinite(p.Y) {
			return nil, &KernelError{Code: ErrNonFiniteCoordinate,
				Message: "vertex coordinates must be finite numbers"}
		}
		if i > 0 && p == pts[i-1] {
			return nil, &KernelError{Code: ErrDegeneratePolygon,
				Message: "consecutive duplicate vertices make the polygon degenerate"}
		}
	}
	n := len(pts)
	if pts[n-1] == pts[0] {
		return nil, &KernelError{Code: ErrDegeneratePolygon,
			Message: "polygon must not be closed with a repeated first vertex"}
	}

	twiceArea := 0.0
	maxCoord := 0.0
	for i := 0; i < n; i++ {
		twiceArea += pts[i].Cross(pts[(i+1)%n])
		maxCoord = max(maxCoord, math.Abs(pts[i].X), math.Abs(pts[i].Y))
	}
	areaTol := eps * max(1.0, maxCoord*maxCoord)
	if math.Abs(twiceArea) <= areaTol {
		return nil, &KernelError{Code: ErrDegeneratePolygon,
			Message: "polygon area is zero: the contour is degenerate"}
	}
	ccw := twiceArea > 0

	// Convexity: every signed turn must have the same sign (zeros allowed
	// for collinear boundary vertices). A single turn of the opposite sign
	// means the contour is concave or self-intersecting.
	tol := eps * max(1.0, maxCoord)
	pos, neg := false, false
	for i := 0; i < n; i++ {
		e1 := pts[i].Sub(pts[(i-1+n)%n])
		e2 := pts[(i+1)%n].Sub(pts[i])
		c := e1.Cross(e2)
		switch {
		case c > tol:
			pos = true
		case c < -tol:
			neg = true
		}
		if pos && neg {
			return nil, &KernelError{Code: ErrNonConvexPolygon,
				Message: "polygon is not convex; only convex polygons are accepted (no convex decomposition)"}
		}
	}
	// Guard against a pathological loop whose turns all read as collinear
	// while the shoelace area was (barely) non-zero.
	if !pos && !neg {
		return nil, &KernelError{Code: ErrDegeneratePolygon,
			Message: "polygon contour is degenerate: no well-defined turns"}
	}
	// A self-intersecting bowtie can have turns of one sign while its edge
	// turns mix with the winding; also reject if the dominant turn sign
	// disagrees with the shoelace orientation.
	if ccw && neg || !ccw && pos {
		return nil, &KernelError{Code: ErrNonConvexPolygon,
			Message: "polygon is not convex; only convex polygons are accepted (no convex decomposition)"}
	}

	return &Polygon{Vertices: pts, CCW: ccw, scale: max(1.0, maxCoord)}, nil
}

// centroid returns the arithmetic mean of the vertices, used only to seed the
// first GJK search direction.
func (p *Polygon) centroid() Vec2 {
	var c Vec2
	for _, v := range p.Vertices {
		c = c.Add(v)
	}
	return c.Scale(1.0 / float64(len(p.Vertices)))
}

// maxLen bounds the coordinate magnitude for tolerance scaling.
func (p *Polygon) maxLen() float64 { return p.scale }

func isFinite(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }
