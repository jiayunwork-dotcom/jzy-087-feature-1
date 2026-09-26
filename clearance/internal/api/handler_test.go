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

type pt struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

func body(a, b []pt) string {
	buf := &bytes.Buffer{}
	_ = json.NewEncoder(buf).Encode(map[string]any{"polygonA": a, "polygonB": b})
	return buf.String()
}

func rectPts(x0, y0, x1, y1 float64) []pt {
	return []pt{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}
}

func TestHealth(t *testing.T) {
	r := Router()
	w := httptest.NewRecorder()
	req := httptest.NewRequest(http.MethodGet, "/health", nil)
	r.ServeHTTP(w, req)
	if w.Code != http.StatusOK {
		t.Fatalf("health status = %d", w.Code)
	}
}

// TestCollide_Separated verifies the known-gap example end to end over HTTP.
func TestCollide_Separated(t *testing.T) {
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
		t.Fatalf("decode: %v body=%s", err, w.Body.String())
	}
	if res.Status != geometry.StatusSeparated {
		t.Fatalf("status = %s", res.Status)
	}
	if d := res.Distance - 0.75; d > 1e-12 || d < -1e-12 {
		t.Fatalf("distance = %.15g", res.Distance)
	}
	if res.Normal.X != 1 || res.Normal.Y != 0 {
		t.Fatalf("normal = %v", res.Normal)
	}
	if res.Iterations == 0 {
		t.Fatalf("iterations should be reported and positive")
	}
}

func TestCollide_Penetrated(t *testing.T) {
	r := Router()
	w := httptest.NewRecorder()
	// x overlap 0.3, y overlap 0.9 -> depth 0.3.
	req := httptest.NewRequest(http.MethodPost, "/collide",
		strings.NewReader(body(rectPts(0, 0, 2, 1), rectPts(1.7, 0.1, 3.7, 1.9))))
	req.Header.Set("Content-Type", "application/json")
	r.ServeHTTP(w, req)
	if w.Code != http.StatusOK {
		t.Fatalf("status = %d body = %s", w.Code, w.Body.String())
	}
	var res geometry.Result
	if err := json.Unmarshal(w.Body.Bytes(), &res); err != nil {
		t.Fatal(err)
	}
	if res.Status != geometry.StatusPenetrated {
		t.Fatalf("status = %s", res.Status)
	}
	if d := res.PenetrationDepth - 0.3; d > 1e-12 || d < -1e-12 {
		t.Fatalf("depth = %.15g, want 0.3", res.PenetrationDepth)
	}
}

func TestCollide_ErrorCases(t *testing.T) {
	r := Router()
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
			raw:        `{"polygonA":[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1}]}`,
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrTooFewVertices,
		},
		{
			name:       "too few vertices",
			raw:        body([]pt{{0, 0}, {1, 1}}, rectPts(0, 0, 1, 1)),
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrTooFewVertices,
		},
		{
			name:       "degenerate polygon",
			raw:        body([]pt{{0, 0}, {1, 1}, {2, 2}}, rectPts(0, 0, 1, 1)),
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrDegeneratePolygon,
		},
		{
			name: "concave polygon refused",
			raw: body([]pt{{0, 0}, {3, 0}, {3, 1}, {1, 1}, {1, 3}, {0, 3}},
				rectPts(0, 0, 1, 1)),
			wantStatus: http.StatusBadRequest,
			wantCode:   geometry.ErrNonConvexPolygon,
		},
		{
			name:       "non numeric coordinate",
			raw:        `{"polygonA":[{"x":"a","y":0},{"x":1,"y":0},{"x":1,"y":1}],"polygonB":[{"x":5,"y":5},{"x":6,"y":5},{"x":6,"y":6}]}`,
			wantStatus: http.StatusBadRequest,
			wantCode:   "INVALID_JSON",
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			w := httptest.NewRecorder()
			req := httptest.NewRequest(http.MethodPost, "/collide", strings.NewReader(tc.raw))
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

func TestCollide_ErrorIdentifiesWhichPolygon(t *testing.T) {
	r := Router()
	w := httptest.NewRecorder()
	// Valid A, degenerate B: the message must name polygon B.
	req := httptest.NewRequest(http.MethodPost, "/collide",
		strings.NewReader(body(rectPts(0, 0, 1, 1), []pt{{0, 0}, {1, 1}, {2, 2}})))
	req.Header.Set("Content-Type", "application/json")
	r.ServeHTTP(w, req)
	if !strings.Contains(w.Body.String(), "polygon B") {
		t.Fatalf("error should identify polygon B: %s", w.Body.String())
	}
}

func TestCollide_MethodNotAllowed(t *testing.T) {
	r := Router()
	w := httptest.NewRecorder()
	req := httptest.NewRequest(http.MethodGet, "/collide", nil)
	r.ServeHTTP(w, req)
	if w.Code == http.StatusOK {
		t.Fatalf("GET /collide should not succeed")
	}
}
