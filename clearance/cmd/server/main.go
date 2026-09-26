// Command clearance-svc serves the assembly-clearance geometric kernel over
// HTTP. It exposes no UI: POST /collide with two convex polygons returns the
// separation or penetration result.
package main

import (
	"log"
	"os"

	"clearance/internal/api"
)

func main() {
	port := os.Getenv("PORT")
	if port == "" {
		port = "8080"
	}
	r := api.Router()
	log.Printf("clearance service listening on :%s", port)
	if err := r.Run(":" + port); err != nil {
		log.Fatalf("server failed: %v", err)
	}
}
