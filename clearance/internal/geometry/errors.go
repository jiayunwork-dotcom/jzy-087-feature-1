package geometry

// Error codes returned by the kernel. They are stable strings exposed to
// HTTP clients.
const (
	// ErrTooFewVertices: a polygon carries fewer than three vertices.
	ErrTooFewVertices = "TOO_FEW_VERTICES"
	// ErrDegeneratePolygon: zero-area contour or duplicate boundary points.
	ErrDegeneratePolygon = "DEGENERATE_POLYGON"
	// ErrNonConvexPolygon: the contour is concave (or self-intersecting).
	ErrNonConvexPolygon = "NON_CONVEX_POLYGON"
	// ErrNonFiniteCoordinate: NaN / +Inf / -Inf input coordinate.
	ErrNonFiniteCoordinate = "NON_FINITE_COORDINATE"
	// ErrInvalidPolygonOrder: a polygon name used in a message is unknown.
	ErrInvalidPolygonOrder = "INVALID_POLYGON_ORDER"
	// ErrGJKNoConvergence: simplex iteration hit its step budget.
	ErrGJKNoConvergence = "GJK_NO_CONVERGENCE"
	// ErrEPANoConvergence: polytope expansion hit its step budget.
	ErrEPANoConvergence = "EPA_NO_CONVERGENCE"
	// ErrEPAFailure: no usable closest edge could be derived.
	ErrEPAFailure = "EPA_FAILURE"
)

// KernelError is a classified error produced by the geometric kernel.
type KernelError struct {
	Code    string
	Message string
}

func (e *KernelError) Error() string { return e.Code + ": " + e.Message }

// namedError attaches which input polygon ("A" or "B") caused the failure.
func namedError(which string, err *KernelError) *KernelError {
	return &KernelError{
		Code:    err.Code,
		Message: "polygon " + which + ": " + err.Message,
	}
}
