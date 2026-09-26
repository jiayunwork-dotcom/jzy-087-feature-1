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

// sweepBody builds a /sweep request body; nil velocities are omitted
// (the service then treats the part as stationary).
func sweepBody(a, b []pt, va, vb *pt) string {
	m := map[string]any{"polygonA": a, "polygonB": b}
	if va != nil {
		m["velocityA"] = va
	}
	if vb != nil {
		m["velocityB"] = vb
	}
	buf := &bytes.Buffer{}
	_ = json.NewEncoder(buf).Encode(m)
	return buf.String()
}

func postSweep(t *testing.T, body string) *httptest.ResponseRecorder {
	t.Helper()
	r := Router()
	w := httptest.NewRecorder()
	req := httptest.NewRequest(http.MethodPost, "/sweep", strings.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	r.ServeHTTP(w, req)
	return w
}

func decodeSweep(t *testing.T, w *httptest.ResponseRecorder) geometry.SweepResult {
	t.Helper()
	if w.Code != http.StatusOK {
		t.Fatalf("status = %d body = %s", w.Code, w.Body.String())
	}
	var res geometry.SweepResult
	if err := json.Unmarshal(w.Body.Bytes(), &res); err != nil {
		t.Fatalf("decode: %v body=%s", err, w.Body.String())
	}
	return res
}

// TestSweep_Impact verifies the head-on closed-form case end to end:
// gap 1.5, closing speed 2, so the time of impact is exactly 0.75 and the
// contact normal is +X.
func TestSweep_Impact(t *testing.T) {
	stationary := pt{0, 0}
	toward := pt{-2, 0}
	w := postSweep(t, sweepBody(rectPts(0, 0, 1, 1), rectPts(2.5, 0, 3.5, 1), &stationary, &toward))
	res := decodeSweep(t, w)

	if res.Status != geometry.StatusImpact {
		t.Fatalf("status = %s, want impact", res.Status)
	}
	if d := res.Time - 0.75; d > 1e-12 || d < -1e-12 {
		t.Fatalf("time of impact = %.15g, want 0.75", res.Time)
	}
	if res.Normal.X != 1 || res.Normal.Y != 0 {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
	if res.Distance != 0 || res.PenetrationDepth != 0 {
		t.Fatalf("impact must report zero gap/depth, got %v/%v", res.Distance, res.PenetrationDepth)
	}
	if d := res.PointA.X - 1; d > 1e-9 || d < -1e-9 {
		t.Fatalf("contact point on A = %v, expected the x=1 edge", res.PointA)
	}
	if res.Iterations == 0 {
		t.Fatalf("iterations should be reported and positive")
	}
}

// TestSweep_Clear: receding motion is safe for the whole window with the
// closest approach exactly at t=0.
func TestSweep_Clear(t *testing.T) {
	stationary := pt{0, 0}
	away := pt{2, 0}
	w := postSweep(t, sweepBody(rectPts(0, 0, 1, 1), rectPts(2.5, 0, 3.5, 1), &stationary, &away))
	res := decodeSweep(t, w)

	if res.Status != geometry.StatusClear {
		t.Fatalf("status = %s, want clear", res.Status)
	}
	if res.Time != 0 {
		t.Fatalf("closest-approach time = %.15g, want exactly 0", res.Time)
	}
	if d := res.Distance - 1.5; d > 1e-12 || d < -1e-12 {
		t.Fatalf("min gap = %.15g, want 1.5", res.Distance)
	}
	if res.Normal.X != 1 || res.Normal.Y != 0 {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
}

// TestSweep_NoVelocitiesMatchesCollide: without velocity fields the sweep
// degenerates to the static query at t=0.
func TestSweep_NoVelocitiesMatchesCollide(t *testing.T) {
	a := rectPts(0, 0, 2, 1)
	b := rectPts(2.75, -0.5, 4.25, 1.5) // known gap 0.75

	r := Router()
	wc := httptest.NewRecorder()
	reqC := httptest.NewRequest(http.MethodPost, "/collide", strings.NewReader(body(a, b)))
	reqC.Header.Set("Content-Type", "application/json")
	r.ServeHTTP(wc, reqC)
	var static geometry.Result
	if err := json.Unmarshal(wc.Body.Bytes(), &static); err != nil {
		t.Fatalf("decode collide: %v", err)
	}

	res := decodeSweep(t, postSweep(t, sweepBody(a, b, nil, nil)))
	if res.Status != geometry.StatusClear || res.Time != 0 {
		t.Fatalf("status/time = %s/%.15g, want clear at 0", res.Status, res.Time)
	}
	if res.Distance != static.Distance || res.Normal != static.Normal {
		t.Fatalf("sweep without velocities = %v/%v, static = %v/%v",
			res.Distance, res.Normal, static.Distance, static.Normal)
	}
	if res.PointA != static.PointA || res.PointB != static.PointB {
		t.Fatalf("closest points %v/%v, static = %v/%v", res.PointA, res.PointB, static.PointA, static.PointB)
	}
}

// TestSweep_InitialPenetration: overlapping at t=0 -> impact at time 0 with
// the static penetration verdict.
func TestSweep_InitialPenetration(t *testing.T) {
	va := pt{5, 5}
	vb := pt{-9, 2}
	// x overlap 0.3, y overlap 0.9 -> depth 0.3.
	w := postSweep(t, sweepBody(rectPts(0, 0, 2, 1), rectPts(1.7, 0.1, 3.7, 1.9), &va, &vb))
	res := decodeSweep(t, w)

	if res.Status != geometry.StatusImpact || res.Time != 0 {
		t.Fatalf("status/time = %s/%.15g, want impact at 0", res.Status, res.Time)
	}
	if d := res.PenetrationDepth - 0.3; d > 1e-12 || d < -1e-12 {
		t.Fatalf("depth = %.15g, want 0.3", res.PenetrationDepth)
	}
}

// TestSweep_WindowEndBoundary pins the t=1 edge: touching exactly at the
// window end is an impact near t=1; stopping 0.1 short is clear with the
// closest approach exactly at t=1.
func TestSweep_WindowEndBoundary(t *testing.T) {
	stationary := pt{0, 0}

	touch := pt{-1.5, 0} // gap 1.5, closes 1.5 -> touch at t=1
	res := decodeSweep(t, postSweep(t, sweepBody(rectPts(0, 0, 1, 1), rectPts(2.5, 0, 3.5, 1), &stationary, &touch)))
	if res.Status != geometry.StatusImpact {
		t.Fatalf("touch-at-1: status = %s, want impact", res.Status)
	}
	if d := res.Time - 1; d > 1e-9 || d < -1e-9 {
		t.Fatalf("touch-at-1: time = %.15g, want ~1", res.Time)
	}

	short := pt{-1.4, 0} // closes 1.4 < 1.5
	res = decodeSweep(t, postSweep(t, sweepBody(rectPts(0, 0, 1, 1), rectPts(2.5, 0, 3.5, 1), &stationary, &short)))
	if res.Status != geometry.StatusClear {
		t.Fatalf("short-at-1: status = %s, want clear", res.Status)
	}
	if res.Time != 1 {
		t.Fatalf("short-at-1: closest time = %.15g, want exactly 1", res.Time)
	}
	if d := res.Distance - 0.1; d > 1e-12 || d < -1e-12 {
		t.Fatalf("short-at-1: min gap = %.15g, want 0.1", res.Distance)
	}
}

// TestSweep_GrazingOverHTTP is the anti-sampling witness over HTTP: the
// overlap window (0.4, 0.45) contains no 1/8-grid sample, yet the service
// must report the impact at t=0.4.
func TestSweep_GrazingOverHTTP(t *testing.T) {
	stationary := pt{0, 0}
	vb := pt{2, -20}
	b := []pt{{-1.8, 8}, {-0.8, 8}, {-0.8, 9}, {-1.8, 9}}
	res := decodeSweep(t, postSweep(t, sweepBody(rectPts(0, 0, 1, 1), b, &stationary, &vb)))
	if res.Status != geometry.StatusImpact {
		t.Fatalf("status = %s, want impact", res.Status)
	}
	if d := res.Time - 0.4; d > 1e-9 || d < -1e-9 {
		t.Fatalf("time of impact = %.15g, want 0.4", res.Time)
	}
	if res.Normal.X != -1 || res.Normal.Y != 0 {
		t.Fatalf("normal = %v, want (-1,0)", res.Normal)
	}
}

func TestSweep_ErrorCases(t *testing.T) {
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
			raw:        `{"polygonA":[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1}],"velocityA":{"x":0,"y":0}}`,
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrTooFewVertices,
		},
		{
			name: "degenerate polygon",
			raw: sweepBody([]pt{{0, 0}, {1, 1}, {2, 2}}, rectPts(0, 0, 1, 1),
				&pt{0, 0}, &pt{0, 0}),
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrDegeneratePolygon,
		},
		{
			name: "concave polygon refused",
			raw: sweepBody([]pt{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}},
				rectPts(0, 0, 1, 1), &pt{0, 0}, &pt{0, 0}),
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrNonConvexPolygon,
		},
		{
			name:       "velocity missing component",
			raw:        `{"polygonA":[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1}],"polygonB":[{"x":2,"y":0},{"x":3,"y":0},{"x":3,"y":1}],"velocityA":{"x":1}}`,
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrNonFiniteCoordinate,
		},
		{
			name:       "velocity non-numeric",
			raw:        `{"polygonA":[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1}],"polygonB":[{"x":2,"y":0},{"x":3,"y":0},{"x":3,"y":1}],"velocityB":{"x":"fast","y":0}}`,
			wantStatus: http.StatusBadRequest,
			wantCode:   "INVALID_JSON",
		},
		{
			name:       "velocity overflows to non-finite",
			raw:        `{"polygonA":[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1}],"polygonB":[{"x":2,"y":0},{"x":3,"y":0},{"x":3,"y":1}],"velocityA":{"x":1e999,"y":0}}`,
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

// TestSweep_ErrorIdentifiesVelocity checks the error message names the
// offending velocity vector.
func TestSweep_ErrorIdentifiesVelocity(t *testing.T) {
	raw := `{"polygonA":[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1}],"polygonB":[{"x":2,"y":0},{"x":3,"y":0},{"x":3,"y":1}],"velocityB":{"y":2}}`
	w := postSweep(t, raw)
	if !strings.Contains(w.Body.String(), "velocity B") {
		t.Fatalf("error should identify velocity B: %s", w.Body.String())
	}
}

// TestSweep_NaNVelocityRejected documents that NaN cannot arrive over JSON
// (the parser rejects it as INVALID_JSON), while the geometry layer rejects
// non-finite velocities directly.
func TestSweep_NaNVelocityRejected(t *testing.T) {
	raw := `{"polygonA":[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1}],"polygonB":[{"x":2,"y":0},{"x":3,"y":0},{"x":3,"y":1}],"velocityA":{"x":NaN,"y":0}}`
	w := postSweep(t, raw)
	if w.Code != http.StatusBadRequest {
		t.Fatalf("status = %d, want 400", w.Code)
	}
}
