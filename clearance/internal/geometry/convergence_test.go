package geometry

import (
	"math"
	"math/rand"
	"sort"
	"testing"
)

// TestIterationsBounded asserts both branches terminate well within their
// declared budgets (so the iteration caps are real safeguards, not the
// expected path).
func TestIterationsBounded(t *testing.T) {
	a := rect(0, 0, 2, 1)
	for i := 0; i < 200; i++ {
		b := rect(float64(i)*0.07-5.0, -3.0, float64(i)*0.07-4.0, 4.0)
		res, err := Evaluate(a, b)
		if err != nil {
			t.Fatalf("i=%d: %v", i, err)
		}
		if res.Iterations <= 0 || res.Iterations > MaxGJKIterations+MaxEPAIterations {
			t.Fatalf("i=%d iterations out of bounds: %d", i, res.Iterations)
		}
	}
}

// TestRandomSeparatedMatchesBruteForce generates random separated convex
// polygons and compares the simplex distance against a brute-force minimum
// distance between all edge pairs. This guards the closest-feature reduction
// without using any bounding-box approximation inside the kernel.
func TestRandomSeparatedMatchesBruteForce(t *testing.T) {
	rng := rand.New(rand.NewSource(42))
	for iter := 0; iter < 300; iter++ {
		// Convex polygon: regular n-gon perturbed slightly.
		a := randomConvex(rng, 3+rng.Intn(5), 3.0)
		b0 := randomConvex(rng, 3+rng.Intn(5), 2.0)
		// Place B well away from A along a random direction.
		theta := rng.Float64() * 2 * math.Pi
		shift := Vec2{X: math.Cos(theta), Y: math.Sin(theta)}.Scale(8.0)
		b := translate(b0, shift)

		res, err := Evaluate(a, b)
		if err != nil {
			t.Fatalf("iter %d: evaluate: %v", iter, err)
		}
		if res.Status != StatusSeparated {
			t.Fatalf("iter %d: expected separated", iter)
		}
		want := bruteForceDistance(a, b)
		if !approxEqRel(res.Distance, want, 1e-7, 1e-9) {
			t.Fatalf("iter %d: simplex distance %.10g != brute-force %.10g",
				iter, res.Distance, want)
		}
		// Witness pair belongs to the polygons and realizes the distance.
		if d := res.PointA.Sub(res.PointB).Len(); !approxEqRel(d, want, 1e-7, 1e-9) {
			t.Fatalf("iter %d: witness distance %.10g != %.10g", iter, d, want)
		}
		if !pointInOrOn(t, a, res.PointA, 1e-7) {
			t.Fatalf("iter %d: pointA %v outside A", iter, res.PointA)
		}
		if !pointInOrOn(t, b, res.PointB, 1e-7) {
			t.Fatalf("iter %d: pointB %v outside B", iter, res.PointB)
		}
	}
}

// TestRandomPenetrationSymmetry checks A vs B and B vs A report equal depth.
func TestRandomPenetrationSymmetry(t *testing.T) {
	rng := rand.New(rand.NewSource(7))
	for iter := 0; iter < 100; iter++ {
		a := randomConvex(rng, 4+rng.Intn(4), 4.0)
		b := randomConvex(rng, 4+rng.Intn(4), 3.5)
		// Center both near the origin so they typically overlap.
		b = translate(b, Vec2{X: rng.NormFloat64() * 0.5, Y: rng.NormFloat64() * 0.5})
		r1, err := Evaluate(a, b)
		if err != nil {
			t.Fatalf("iter %d: %v", iter, err)
		}
		if r1.Status != StatusPenetrated {
			continue
		}
		r2, err := Evaluate(b, a)
		if err != nil {
			t.Fatalf("iter %d swapped: %v", iter, err)
		}
		if !approxEqRel(r1.PenetrationDepth, r2.PenetrationDepth, 1e-8, 1e-9) {
			t.Fatalf("iter %d: asymmetric depth %.10g vs %.10g",
				iter, r1.PenetrationDepth, r2.PenetrationDepth)
		}
	}
}

// randomConvex builds a guaranteed-convex polygon: random points in a disk,
// reduced to their convex hull (kept as a test-only construction helper;
// the service itself never performs convex decomposition).
func randomConvex(rng *rand.Rand, n int, radius float64) []Vec2 {
	for {
		pts := make([]Vec2, 3*n)
		for i := range pts {
			r := radius * math.Sqrt(rng.Float64())
			th := rng.Float64() * 2 * math.Pi
			pts[i] = Vec2{X: r * math.Cos(th), Y: r * math.Sin(th)}
		}
		hull := convexHullOf(pts)
		if len(hull) >= n {
			if len(hull) > 2*n {
				hull = hull[:2*n]
			}
			return hull
		}
	}
}

// convexHullOf is a test-only monotone chain hull for Vec2 points.
func convexHullOf(pts []Vec2) []Vec2 {
	sorted := append([]Vec2(nil), pts...)
	sort.Slice(sorted, func(i, j int) bool {
		if sorted[i].X != sorted[j].X {
			return sorted[i].X < sorted[j].X
		}
		return sorted[i].Y < sorted[j].Y
	})
	cross := func(o, a, b Vec2) float64 { return a.Sub(o).Cross(b.Sub(o)) }
	lower := []Vec2{}
	for _, p := range sorted {
		for len(lower) >= 2 && cross(lower[len(lower)-2], lower[len(lower)-1], p) <= 0 {
			lower = lower[:len(lower)-1]
		}
		lower = append(lower, p)
	}
	upper := []Vec2{}
	for i := len(sorted) - 1; i >= 0; i-- {
		p := sorted[i]
		for len(upper) >= 2 && cross(upper[len(upper)-2], upper[len(upper)-1], p) <= 0 {
			upper = upper[:len(upper)-1]
		}
		upper = append(upper, p)
	}
	return append(lower[:len(lower)-1], upper[:len(upper)-1]...)
}

// bruteForceDistance computes min over all vertex-edge pair distances between
// two convex polygons. Used only as an independent test oracle.
func bruteForceDistance(a, b []Vec2) float64 {
	best := math.Inf(1)
	for i := 0; i < len(a); i++ {
		for j := 0; j < len(b); j++ {
			d := segmentSegment(
				a[i], a[(i+1)%len(a)],
				b[j], b[(j+1)%len(b)],
			)
			if d < best {
				best = d
			}
		}
	}
	return best
}

func segmentSegment(p1, p2, q1, q2 Vec2) float64 {
	return math.Min(
		math.Min(pointSegment(p1, q1, q2), pointSegment(p2, q1, q2)),
		math.Min(pointSegment(q1, p1, p2), pointSegment(q2, p1, p2)),
	)
}

func pointSegment(p, a, b Vec2) float64 {
	e := b.Sub(a)
	if e.Len2() == 0 {
		return p.Sub(a).Len()
	}
	t := clamp01(p.Sub(a).Dot(e) / e.Len2())
	q := a.Add(e.Scale(t))
	return p.Sub(q).Len()
}

func clamp01(t float64) float64 {
	if t < 0 {
		return 0
	}
	if t > 1 {
		return 1
	}
	return t
}
