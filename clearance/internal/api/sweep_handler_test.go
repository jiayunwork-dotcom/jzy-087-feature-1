package api

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"clearance/internal/geometry"
)

type vec struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

func sweepBodyRaw(raw string) *strings.Reader { return strings.NewReader(raw) }

func sweepReq(a, b []pt, va, vb vec, extra string) string {
	body := map[string]any{
		"polygonA":  a,
		"polygonB":  b,
		"velocityA": va,
		"velocityB": vb,
	}
	_ = extra
	buf := &strings.Builder{}
	enc := json.NewEncoder(buf)
	_ = enc.Encode(body)
	return buf.String()
}

func TestSweep_Contact(t *testing.T) {
	r := Router()
	w := httptest.NewRecorder()
	req := httptest.NewRequest(http.MethodPost, "/sweep",
		sweepBodyRaw(sweepReq(
			rectPts(0, 0, 1, 1),
			rectPts(1.5, 0, 2.5, 1),
			vec{}, vec{X: -2}, "",
		)))
	req.Header.Set("Content-Type", "application/json")
	r.ServeHTTP(w, req)

	if w.Code != http.StatusOK {
		t.Fatalf("status = %d body = %s", w.Code, w.Body.String())
	}
	var res geometry.SweepResult
	if err := json.Unmarshal(w.Body.Bytes(), &res); err != nil {
		t.Fatalf("decode: %v body=%s", err, w.Body.String())
	}
	if res.Status != geometry.StatusContact {
		t.Fatalf("status = %s, want contact", res.Status)
	}
	if d := res.Time - 0.25; d > 1e-9 || d < -1e-9 {
		t.Fatalf("time = %.15g, want 0.25", res.Time)
	}
	if res.Normal.X != 1 || res.Normal.Y != 0 {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
	if res.Iterations == 0 {
		t.Fatalf("iterations should be reported and positive")
	}
}

func TestSweep_SafeSeparating(t *testing.T) {
	r := Router()
	w := httptest.NewRecorder()
	req := httptest.NewRequest(http.MethodPost, "/sweep",
		sweepBodyRaw(sweepReq(
			rectPts(0, 0, 1, 1),
			rectPts(1.5, 0, 2.5, 1),
			vec{}, vec{X: 2}, "",
		)))
	req.Header.Set("Content-Type", "application/json")
	r.ServeHTTP(w, req)
	if w.Code != http.StatusOK {
		t.Fatalf("status = %d body = %s", w.Code, w.Body.String())
	}
	var res geometry.SweepResult
	if err := json.Unmarshal(w.Body.Bytes(), &res); err != nil {
		t.Fatal(err)
	}
	if res.Status != geometry.StatusSafe {
		t.Fatalf("status = %s, want safe", res.Status)
	}
	if res.Time != 0 {
		t.Fatalf("closest time = %v, want 0", res.Time)
	}
	if d := res.Distance - 0.5; d > 1e-12 || d < -1e-12 {
		t.Fatalf("distance = %.15g, want 0.5", res.Distance)
	}
}

func TestSweep_OffsetsAndCommonMotion(t *testing.T) {
	// Local unit squares for both; B offset to x=1.5, A offset to (10,-3),
	// both share velocity (5,5) and B additionally closes at (-2,0).
	raw := `{
	  "polygonA":[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1},{"x":0,"y":1}],
	  "polygonB":[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1},{"x":0,"y":1}],
	  "offsetA":{"x":10,"y":-3},
	  "offsetB":{"x":11.5,"y":-3},
	  "velocityA":{"x":5,"y":5},
	  "velocityB":{"x":3,"y":5}
	}`
	r := Router()
	w := httptest.NewRecorder()
	req := httptest.NewRequest(http.MethodPost, "/sweep", strings.NewReader(raw))
	req.Header.Set("Content-Type", "application/json")
	r.ServeHTTP(w, req)
	if w.Code != http.StatusOK {
		t.Fatalf("status = %d body = %s", w.Code, w.Body.String())
	}
	var res geometry.SweepResult
	if err := json.Unmarshal(w.Body.Bytes(), &res); err != nil {
		t.Fatal(err)
	}
	if res.Status != geometry.StatusContact {
		t.Fatalf("status = %s, want contact", res.Status)
	}
	if d := res.Time - 0.25; d > 1e-9 || d < -1e-9 {
		t.Fatalf("time = %.15g, want 0.25 (relative motion must be invariant)", res.Time)
	}
	if res.Normal.X != 1 || res.Normal.Y != 0 {
		t.Fatalf("normal = %v, want (1,0)", res.Normal)
	}
}

func TestSweep_ErrorCases(t *testing.T) {
	r := Router()
	good := rectPts(0, 0, 1, 1)
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
			name: "missing velocity B",
			raw: `{"polygonA":` + jsonArr(good) + `,"polygonB":` + jsonArr(good) +
				`,"velocityA":{"x":0,"y":0}}`,
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrNonFiniteVelocity,
		},
		{
			name: "velocity component not numeric",
			raw: `{"polygonA":` + jsonArr(good) + `,"polygonB":` + jsonArr(good) +
				`,"velocityA":{"x":"fast","y":0},"velocityB":{"x":0,"y":0}}`,
			wantStatus: http.StatusBadRequest,
			wantCode:   "INVALID_JSON",
		},
		{
			name: "too few vertices",
			raw: `{"polygonA":[{"x":0,"y":0},{"x":1,"y":1}],"polygonB":` + jsonArr(good) +
				`,"velocityA":{"x":0,"y":0},"velocityB":{"x":0,"y":0}}`,
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrTooFewVertices,
		},
		{
			name: "concave polygon refused",
			raw: `{"polygonA":[{"x":0,"y":0},{"x":3,"y":0},{"x":3,"y":1},{"x":1,"y":1},{"x":1,"y":3},{"x":0,"y":3}]` +
				`,"polygonB":` + jsonArr(good) +
				`,"velocityA":{"x":0,"y":0},"velocityB":{"x":0,"y":0}}`,
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrNonConvexPolygon,
		},
		{
			name: "non numeric offset",
			raw: `{"polygonA":` + jsonArr(good) + `,"polygonB":` + jsonArr(good) +
				`,"offsetA":{"x":null,"y":0}` +
				`,"velocityA":{"x":0,"y":0},"velocityB":{"x":0,"y":0}}`,
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrNonFiniteCoordinate,
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			w := httptest.NewRecorder()
			req := httptest.NewRequest(http.MethodPost, "/sweep", strings.NewReader(tc.raw))
			req.Header.Set("Content-Type", "application/json")
			r.ServeHTTP(w, req)
			if w.Code != tc.wantStatus {
				t.Fatalf("status = %d body = %s, want %d", w.Code, w.Body.String(), tc.wantStatus)
			}
			var er struct {
				Code    string `json:"code"`
				Message string `json:"message"`
			}
			if err := json.Unmarshal(w.Body.Bytes(), &er); err != nil {
				t.Fatalf("decode: %v", err)
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

// TestSweep_OldCollideUnchanged ensures adding the new entry point did not
// alter the old query: it accepts bodies without velocity fields and returns
// the original response shape.
func TestSweep_OldCollideUnchanged(t *testing.T) {
	r := Router()
	w := httptest.NewRecorder()
	req := httptest.NewRequest(http.MethodPost, "/collide",
		strings.NewReader(body(rectPts(0, 0, 2, 1), rectPts(2.75, -0.5, 4.25, 1.5))))
	req.Header.Set("Content-Type", "application/json")
	r.ServeHTTP(w, req)
	if w.Code != http.StatusOK {
		t.Fatalf("status = %d body = %s", w.Code, w.Body.String())
	}
	var raw map[string]any
	if err := json.Unmarshal(w.Body.Bytes(), &raw); err != nil {
		t.Fatal(err)
	}
	if raw["status"] != "separated" {
		t.Fatalf("status = %v", raw["status"])
	}
	if _, ok := raw["time"]; ok {
		t.Fatalf("old /collide response must not carry sweep-only field time: %v", raw)
	}
	if _, ok := raw["velocityA"]; ok {
		t.Fatalf("old /collide response must not echo request fields")
	}
}

func TestSweep_MethodNotAllowed(t *testing.T) {
	r := Router()
	w := httptest.NewRecorder()
	req := httptest.NewRequest(http.MethodGet, "/sweep", nil)
	r.ServeHTTP(w, req)
	if w.Code == http.StatusOK {
		t.Fatalf("GET /sweep should not succeed")
	}
}

// jsonArr encodes a point array for inline JSON fixtures.
func jsonArr(pts []pt) string {
	b, _ := json.Marshal(pts)
	return string(b)
}
