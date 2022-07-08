# HOW TO PROTOBUF

The definition files can be found in `commonroad/scenario_definition/protobuf_format`. The file 
`commonroad.proto` contains the root of the whole message and imports all submessages which are divided 
and stored in multiple files with ending `.proto`.

Executable scripts for serializing and deserializing can be generated based on the previous mentioned
definition files. The scripts are stored in `commonroad/scenario_definition/protobuf_format/generated_scripts`. 
After modifying the definition files the corresponding scripts have to be generated immediately to be consistent.

The following command can be used to update the scripts:
```
protoc --proto_path=./commonroad-io/commonroad/scenario_definition/protobuf_format/definition_files/ --python_out=./commonroad-io/commonroad/scenario_definition/protobuf_format/generated_scripts/ ./commonroad-io/commonroad/scenario_definition/protobuf_format/definition_files/*.proto
```

Maybe some import paths have to be adjusted.
We recommend to use libprotoc 3.6.1 for generating the Python files (you can check the version via ```protoc --version```)