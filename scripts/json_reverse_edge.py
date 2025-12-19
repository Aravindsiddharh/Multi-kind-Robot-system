import json

# Load existing GeoJSON
input_file = "/home/siddhu/route_ws/src/bcr_bot/hospital_route.geojson"
output_file = "/home/siddhu/route_ws/src/bcr_bot/hospital_route_updated.geojson"

with open(input_file, "r") as f:
    geojson = json.load(f)

# List of edges missing reverse
missing_reverse = [
(12, 10),
(26, 27),
(5, 4),
(21, 22),
(30, 0),
(22, 23),
(3, 13),
(9, 8),
(10, 9),
(28, 30),
(13, 14),
(6, 5),
(24, 26),
(15, 20),
(18, 19),
(12, 21),
(14, 15),
(23, 24),
(0, 1),
(1, 2),
(10, 11),
(19, 17),
(28, 29),
(16, 12),
(24, 25),
(7, 6),
(26, 28),
(20, 18),
(4, 13),
(8, 7),
(17, 16),
(2, 3),
(30, 31),


]

# Map node id to coordinates for easy lookup
id_to_coord = {}
for feature in geojson["features"]:
    geom_type = feature["geometry"]["type"]
    prop_id = feature["properties"]["id"]
    if geom_type == "Point":
        id_to_coord[prop_id] = feature["geometry"]["coordinates"]

# Create a set of existing edges to avoid duplicates
existing_edges = set(
    (f["properties"]["startid"], f["properties"]["endid"]) 
    for f in geojson["features"] 
    if "startid" in f["properties"]
)

# Find current max feature id
max_id = max(f["properties"]["id"] for f in geojson["features"])

# Add missing reverse edges
added_count = 0
for start, end in missing_reverse:
    if start in id_to_coord and end in id_to_coord:
        # Skip if reverse edge already exists
        if (end, start) in existing_edges:
            continue

        max_id += 1
        new_edge = {
            "type": "Feature",
            "properties": {
                "id": max_id,
                "startid": end,   # reversed
                "endid": start    # reversed
            },
            "geometry": {
                "type": "MultiLineString",
                "coordinates": [[id_to_coord[end], id_to_coord[start]]]
            }
        }
        geojson["features"].append(new_edge)
        existing_edges.add((end, start))
        added_count += 1

# Save updated GeoJSON with line-by-line formatting
with open(output_file, "w") as f:
    json.dump(geojson, f, indent=4, separators=(',', ': '))

print(f"Added {added_count} reverse edges. Saved as {output_file}")
