import json
from jsonschema import validate
import sys

# sys.argv = ["", "scene.schema.json", "test_scene.json" ]

with open(sys.argv[1]) as f:
    schema = json.load(f)

with open(sys.argv[2]) as f:
    instance = json.load(f)

if validate(instance, schema) is None:
    print("ok")