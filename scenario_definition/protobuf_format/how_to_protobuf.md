# HOW TO PROTOBUF

The .proto definition files can be found in `commonroad/scenario_definition/protobuf_format`, where they have been mapped to 
their respective folders `common`,`dynamic`,`map` and `scenario`.

Executable scripts for serializing and deserializing can be generated based on the previously mentioned
definition files.<br>The scripts can be found in `commonroad/common/protobuf/`.
<br>Just like the definition files, they have been stored in their respective folders.

After modifying the definition files, the corresponding scripts have to be generated immediately to be consistent.

To generate scripts, from `commonroad/scenario_definition/protobuf_format` run the next line of code:
```
./generate_pb.sh
```
If there is an error in the .proto files, it will be thrown and the scripts will not be generated.<br>
If all the .proto files are valid, new protobuf scripts will automatically be generated.

We recommend to use libprotoc 3.6.1 for generating the Python files (you can check the version via ```protoc --version```)