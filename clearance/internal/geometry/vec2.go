// Package geometry implements the assembly-clearance geometric kernel:
// validation of convex polygons and a GJK/EPA evaluation on the Minkowski
// difference of two polygons.
package geometry

import "math"

// Vec2 is a planar vector / point.
type Vec2 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// Add returns a+b.
func (a Vec2) Add(b Vec2) Vec2 { return Vec2{a.X + b.X, a.Y + b.Y} }

// Sub returns a-b.
func (a Vec2) Sub(b Vec2) Vec2 { return Vec2{a.X - b.X, a.Y - b.Y} }

// Scale returns a*s.
func (a Vec2) Scale(s float64) Vec2 { return Vec2{a.X * s, a.Y * s} }

// Dot returns the scalar product a·b.
func (a Vec2) Dot(b Vec2) float64 { return a.X*b.X + a.Y*b.Y }

// Cross returns the z component of the 3D cross product.
func (a Vec2) Cross(b Vec2) float64 { return a.X*b.Y - a.Y*b.X }

// Len returns the Euclidean norm.
func (a Vec2) Len() float64 { return math.Hypot(a.X, a.Y) }

// Len2 returns the squared Euclidean norm.
func (a Vec2) Len2() float64 { return a.X*a.X + a.Y*a.Y }

// Normalized returns the unit vector. It must not be called on a zero vector.
func (a Vec2) Normalized() Vec2 {
	l := a.Len()
	return Vec2{a.X / l, a.Y / l}
}

// PerpLeft returns the 90° counter-clockwise rotation of a (left normal):
// (x,y) -> (-y,x).
func (a Vec2) PerpLeft() Vec2 { return Vec2{-a.Y, a.X} }

// cleanZeros turns signed-zero components into +0 so serialized results do
// not contain "-0".
func (a Vec2) cleanZeros() Vec2 {
	if a.X == 0 {
		a.X = 0
	}
	if a.Y == 0 {
		a.Y = 0
	}
	return a
}

// epsilon returns an absolute tolerance adapted to the magnitudes of the
// operands. The 1.0 floor keeps it meaningful for unit-sized coordinates.
func epsFor(a, b float64) float64 {
	m := math.Max(math.Abs(a), math.Abs(b))
	if m < 1 {
		m = 1
	}
	return eps * m
}

const (
	// eps is the dimensionless relative tolerance used across the kernel.
	eps = 1e-10
	// convergenceRel is the relative EPA expansion threshold: a support
	// vertex must lie at least this fraction (of the polytope size) beyond
	// the current closest edge for the expansion to continue.
	convergenceRel = 1e-9
	// paramTol is the tolerance on segment parameters t in [0,1] when
	// deciding whether a closest point is an endpoint or an interior point.
	paramTol = 1e-10
)
