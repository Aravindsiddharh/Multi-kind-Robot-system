import json

# Load existing GeoJSON
input_file = "/home/siddhu/route_ws/src/bcr_bot/hospital_route.geojson"
with open(input_file, "r") as f:
    geojson = json.load(f)

# Map node id to coordinates for easy lookup (only points)
id_to_coord = {f["properties"]["id"]: f["geometry"]["coordinates"]
               for f in geojson["features"] if f["geometry"]["type"] == "Point"}

# Collect all existing edges (startid, endid)
existing_edges = set(
    (f["properties"]["startid"], f["properties"]["endid"])
    for f in geojson["features"]
    if "startid" in f["properties"]
)

# Find edges that are missing their reverse
missing_reverse_edges = []
for start, end in existing_edges:
    if (end, start) not in existing_edges:
        missing_reverse_edges.append((start, end))

print(f"Total edges in GeoJSON: {len(existing_edges)}")
print(f"Edges missing reverse: {len(missing_reverse_edges)}")
print("List of edges that need reverse:")
for edge in missing_reverse_edges:
    print(edge)
