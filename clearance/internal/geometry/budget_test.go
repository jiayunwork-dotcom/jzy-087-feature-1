package geometry

import "testing"

// TestIterationBudgetExhaustedReportsError proves the iteration caps are
// hard errors rather than infinite loops or a fake "distance zero, separated"
// answer.
func TestIterationBudgetExhaustedReportsError(t *testing.T) {
	a := rect(0, 0, 2, 1)
	b := rect(3, 0, 4, 1)

	// GJK budget exhausted -> GJK_NO_CONVERGENCE.
	origGJK := gjkMaxSteps
	gjkMaxSteps = 0
	_, err := Evaluate(a, b)
	gjkMaxSteps = origGJK
	assertCode(t, err, ErrGJKNoConvergence)

	// Overlapping pair: EPA budget exhausted -> EPA_NO_CONVERGENCE.
	bOverlap := rect(1.5, 0, 3, 1)
	origEPA := epaMaxSteps
	epaMaxSteps = 0
	_, err = Evaluate(a, bOverlap)
	epaMaxSteps = origEPA
	assertCode(t, err, ErrEPANoConvergence)
}

// TestVertexVertexTouch is the boundary-contact case: a single point of A
// touches a single point of B. It must be reported via the penetration
// branch with depth ~0 (the EPA bootstrap must handle a point/seed on the
// origin), never as separated-with-zero-distance.
func TestVertexVertexTouch(t *testing.T) {
	// A occupies x in [0,1]; B occupies x in [1,2]; they share only the
	// point (1,0) on A's top-right / B's bottom-left region. Use squares
	// touching at a single corner.
	a := rect(0, 0, 1, 1)
	b := rect(1, 1, 2, 2)
	res, err := Evaluate(a, b)
	if err != nil {
		t.Fatalf("evaluate: %v", err)
	}
	if res.Status != StatusPenetrated {
		t.Fatalf("status = %s, want penetrated (point contact)", res.Status)
	}
	if res.PenetrationDepth > 1e-8 {
		t.Fatalf("point-contact depth = %.3e, want 0", res.PenetrationDepth)
	}
	if res.Normal.Len() != 0 && abs(res.Normal.Len()-1) > 1e-12 {
		t.Fatalf("contact normal must be unit: %v", res.Normal)
	}
}

// TestEdgeEdgeTouch covers the flat-contact boundary: two rectangles share a
// whole edge. Depth must be ~0 with a normal perpendicular to that edge.
func TestEdgeEdgeTouch(t *testing.T) {
	a := rect(0, 0, 1, 1)
	b := rect(1, 0, 2, 1)
	res, err := Evaluate(a, b)
	if err != nil {
		t.Fatalf("evaluate: %v", err)
	}
	if res.Status != StatusPenetrated {
		t.Fatalf("status = %s, want penetrated (edge contact)", res.Status)
	}
	if res.PenetrationDepth > 1e-8 {
		t.Fatalf("edge-contact depth = %.3e, want 0", res.PenetrationDepth)
	}
	if n := res.Normal; abs(n.X)-1 > 1e-8 || abs(n.Y) > 1e-8 {
		t.Fatalf("edge-contact normal = %v, want (±1,0)", n)
	}
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}
