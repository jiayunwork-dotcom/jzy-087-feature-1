package api

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"clearance/internal/geometry"
)

// sweepBody builds a /sweep request body. Nil maps are omitted.
func sweepBody(a, b []pt, extra map[string]any) string {
	m := map[string]any{"polygonA": a, "polygonB": b}
	for k, v := range extra {
		m[k] = v
	}
	buf := &bytes.Buffer{}
	_ = json.NewEncoder(buf).Encode(m)
	return buf.String()
}

func postSweep(t *testing.T, raw string) *httptest.ResponseRecorder {
	t.Helper()
	r := Router()
	w := httptest.NewRecorder()
	req := httptest.NewRequest(http.MethodPost, "/sweep", strings.NewReader(raw))
	req.Header.Set("Content-Type", "application/json")
	r.ServeHTTP(w, req)
	return w
}

// TestSweepEndpoint_HeadOn exercises the new query end to end over HTTP:
// gap 2, closing speed 4 -> impact at t=0.5 with normal +X.
func TestSweepEndpoint_HeadOn(t *testing.T) {
	w := postSweep(t, sweepBody(rectPts(0, 0, 1, 1), rectPts(3, 0, 4, 1),
		map[string]any{"velocityB": map[string]float64{"x": -4, "y": 0}}))
	if w.Code != http.StatusOK {
		t.Fatalf("status = %d body = %s", w.Code, w.Body.String())
	}
	var res geometry.SweepResult
	if err := json.Unmarshal(w.Body.Bytes(), &res); err != nil {
		t.Fatalf("decode: %v body=%s", err, w.Body.String())
	}
	if res.Verdict != geometry.VerdictContact {
		t.Fatalf("verdict = %s, want contact", res.Verdict)
	}
	if d := res.TimeOfImpact - 0.5; d > 1e-12 || d < -1e-12 {
		t.Fatalf("timeOfImpact = %.15g, want 0.5", res.TimeOfImpact)
	}
	if res.Normal.X != 1 || res.Normal.Y != 0 {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
	if res.Iterations == 0 {
		t.Fatalf("iterations should be reported and positive")
	}
}

// TestSweepEndpoint_Safe: receding parts report safe with the closest
// approach at t=0.
func TestSweepEndpoint_Safe(t *testing.T) {
	w := postSweep(t, sweepBody(rectPts(0, 0, 1, 1), rectPts(3, 0, 4, 1),
		map[string]any{"velocityB": map[string]float64{"x": 4, "y": 0}}))
	if w.Code != http.StatusOK {
		t.Fatalf("status = %d body = %s", w.Code, w.Body.String())
	}
	var res geometry.SweepResult
	if err := json.Unmarshal(w.Body.Bytes(), &res); err != nil {
		t.Fatalf("decode: %v", err)
	}
	if res.Verdict != geometry.VerdictSafe {
		t.Fatalf("verdict = %s, want safe", res.Verdict)
	}
	if res.TimeOfClosestApproach != 0 {
		t.Fatalf("timeOfClosestApproach = %g, want 0", res.TimeOfClosestApproach)
	}
	if d := res.MinDistance - 2; d > 1e-12 || d < -1e-12 {
		t.Fatalf("minDistance = %.15g, want 2", res.MinDistance)
	}
}

// TestSweepEndpoint_OffsetsAndVelocities checks the initial-offset plumbing:
// both shapes given unshifted, offsets supplied separately, and both parts
// moving. The relative problem equals the plain head-on case shifted by
// (10,20), so the impact time and normal are unchanged and the contact
// point moves by the common offset.
func TestSweepEndpoint_OffsetsAndVelocities(t *testing.T) {
	w := postSweep(t, sweepBody(rectPts(0, 0, 1, 1), rectPts(3, 0, 4, 1),
		map[string]any{
			"offsetA":   map[string]float64{"x": 10, "y": 20},
			"offsetB":   map[string]float64{"x": 10, "y": 20},
			"velocityA": map[string]float64{"x": 1, "y": 0},
			"velocityB": map[string]float64{"x": -3, "y": 0},
		}))
	if w.Code != http.StatusOK {
		t.Fatalf("status = %d body = %s", w.Code, w.Body.String())
	}
	var res geometry.SweepResult
	if err := json.Unmarshal(w.Body.Bytes(), &res); err != nil {
		t.Fatalf("decode: %v", err)
	}
	if res.Verdict != geometry.VerdictContact {
		t.Fatalf("verdict = %s, want contact", res.Verdict)
	}
	if d := res.TimeOfImpact - 0.5; d > 1e-12 || d < -1e-12 {
		t.Fatalf("timeOfImpact = %.15g, want 0.5", res.TimeOfImpact)
	}
	if res.Normal.X != 1 || res.Normal.Y != 0 {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
	// Contact face x=1 shifted by offset 10 plus velocityA 1*0.5.
	if d := res.PointA.X - 11.5; d > 1e-9 || d < -1e-9 {
		t.Fatalf("contact x = %.15g, want 11.5", res.PointA.X)
	}
	if d := res.PointA.Y - 20.5; d > 1e-9 || d < -1e-9 {
		t.Fatalf("contact y = %.15g, want 20.5", res.PointA.Y)
	}
}

// TestSweepEndpoint_Errors maps the sweep-specific bad inputs to the same
// error taxonomy as the static query.
func TestSweepEndpoint_Errors(t *testing.T) {
	cases := []struct {
		name       string
		raw        string
		wantStatus int
		wantCode   string
	}{
		{
			name:       "malformed json",
			raw:        `{"polygonA": }`,
			wantStatus: http.StatusBadRequest,
			wantCode:   "INVALID_JSON",
		},
		{
			name:       "missing polygon B",
			raw:        `{"polygonA":[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1}],"velocityA":{"x":1,"y":0}}`,
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrTooFewVertices,
		},
		{
			name: "degenerate polygon",
			raw: sweepBody([]pt{{0, 0}, {1, 1}, {2, 2}}, rectPts(5, 5, 6, 6),
				nil),
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrDegeneratePolygon,
		},
		{
			name: "concave polygon refused",
			raw: sweepBody([]pt{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}}, rectPts(5, 5, 6, 6),
				nil),
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrNonConvexPolygon,
		},
		{
			name: "velocity with missing component",
			raw: sweepBody(rectPts(0, 0, 1, 1), rectPts(5, 5, 6, 6),
				map[string]any{"velocityA": map[string]float64{"x": 1}}),
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrNonFiniteCoordinate,
		},
		{
			name:       "velocity with non-numeric component",
			raw:        `{"polygonA":[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1}],"polygonB":[{"x":5,"y":5},{"x":6,"y":5},{"x":6,"y":6}],"velocityB":{"x":"fast","y":0}}`,
			wantStatus: http.StatusBadRequest,
			wantCode:   "INVALID_JSON",
		},
		{
			name:       "velocity out of float range",
			raw:        `{"polygonA":[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1}],"polygonB":[{"x":5,"y":5},{"x":6,"y":5},{"x":6,"y":6}],"velocityB":{"x":1e999,"y":0}}`,
			wantStatus: http.StatusBadRequest,
			wantCode:   "INVALID_JSON",
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			w := postSweep(t, tc.raw)
			if w.Code != tc.wantStatus {
				t.Fatalf("status = %d body = %s, want %d", w.Code, w.Body.String(), tc.wantStatus)
			}
			var er struct {
				Code    string `json:"code"`
				Message string `json:"message"`
			}
			if err := json.Unmarshal(w.Body.Bytes(), &er); err != nil {
				t.Fatalf("decode error body: %v", err)
			}
			if er.Code != tc.wantCode {
				t.Fatalf("code = %s, want %s", er.Code, tc.wantCode)
			}
			if er.Message == "" {
				t.Fatalf("error message is empty")
			}
		})
	}
}

// TestSweepEndpoint_ResponseShape pins the JSON field names of the new
// query so the contract is explicit.
func TestSweepEndpoint_ResponseShape(t *testing.T) {
	w := postSweep(t, sweepBody(rectPts(0, 0, 1, 1), rectPts(3, 0, 4, 1),
		map[string]any{"velocityB": map[string]float64{"x": -4, "y": 0}}))
	if w.Code != http.StatusOK {
		t.Fatalf("status = %d", w.Code)
	}
	var m map[string]any
	if err := json.Unmarshal(w.Body.Bytes(), &m); err != nil {
		t.Fatalf("decode: %v", err)
	}
	for _, key := range []string{
		"verdict", "timeOfImpact", "timeOfClosestApproach", "minDistance",
		"penetrationDepth", "normal", "pointA", "pointB", "iterations",
	} {
		if _, ok := m[key]; !ok {
			t.Fatalf("response missing key %q: %s", key, w.Body.String())
		}
	}
}

// TestCollide_UnaffectedBySweep pins the static endpoint's contract after
// the sweep addition: same route, same response fields, same numbers.
func TestCollide_UnaffectedBySweep(t *testing.T) {
	r := Router()
	w := httptest.NewRecorder()
	req := httptest.NewRequest(http.MethodPost, "/collide",
		strings.NewReader(body(rectPts(0, 0, 2, 1), rectPts(2.75, -0.5, 4.25, 1.5))))
	req.Header.Set("Content-Type", "application/json")
	r.ServeHTTP(w, req)
	if w.Code != http.StatusOK {
		t.Fatalf("status = %d body = %s", w.Code, w.Body.String())
	}
	var res geometry.Result
	if err := json.Unmarshal(w.Body.Bytes(), &res); err != nil {
		t.Fatalf("decode: %v", err)
	}
	if res.Status != geometry.StatusSeparated || res.Distance != 0.75 ||
		res.Normal.X != 1 || res.Normal.Y != 0 {
		t.Fatalf("static contract changed: %s", w.Body.String())
	}
	// The static response must not grow sweep fields.
	if strings.Contains(w.Body.String(), "timeOfImpact") ||
		strings.Contains(w.Body.String(), "verdict") {
		t.Fatalf("static response polluted with sweep fields: %s", w.Body.String())
	}
}
