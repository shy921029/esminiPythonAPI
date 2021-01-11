# esminiPythonAPI
python api for esminiLib and esminiRMLib

## esmini
esmini is a basic OpenSCENARIO player

It contains the following main libraries:

- RoadManager (esminiRMLib). A library providing an interface to road networks described in the OpenDRIVE format.
- ScenarioEngine (esminiLib). The main library providing a viewer and API interface to traffic scenarios described in the OpenSCENARIO format. This library includes RoadManager.

and a few applications that can be used as is or provide ideas for customized solutions:

- esmini. A scenario player application linking esmini modules statically.
- esmini-dyn. A minimalistic example using the esminiLib to play OpenSCENARIO files.
- odrplot. Produces a data file from OpenDRIVE for plotting the road network in Python.
- odrviewer. Visualize OpenDRIVE road network with populated dummy traffic.
- replayer. Re-play previously executed scenarios.
- osireceiver. A simple application receiving OSI messages from esmini over UDP.

Repository: https://github.com/esmini/esmini
