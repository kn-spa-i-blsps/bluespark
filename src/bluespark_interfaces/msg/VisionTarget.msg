std_msgs/Header header

# Who produced this detection: "pipeline" | "path_marker" | "bin"
# Lets the aggregator decide which source leads and which only reports.
string source

# Whether anything was detected in this frame
bool detected

# --- common geometry for centering (shared by all tasks) ---
float32 offset_x        # horizontal offset from center, normalized -1..1
float32 offset_y        # vertical offset from center, normalized -1..1
float32 angle_deg       # orientation of the target / line
float32 area_fraction   # proximity: fraction of frame area occupied

# --- task-specific fields (0 / empty when unused) ---
string  symbol          # e.g. "flame" / "drop" for bin, "" otherwise
float32 confidence      # general detection confidence

# path marker arrow direction (valid flag guards the unused case)
float32 tip_direction
bool    tip_direction_valid

# --- bounding box in pixels (0 when unused) ---
int32 bbox_x
int32 bbox_y
int32 bbox_w
int32 bbox_h