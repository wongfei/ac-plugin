# ac-plugin

Example of native C++ plugin for Assetto Corsa.

Shows how to write custom game logic and mod physics.

Provides full access to game internals.

Requires VS2015 community + VS2013 (v120) platform toolset.

## Installing

Build & Copy acplugin.dll to game dir:

`d:\steam\steamapps\common\assettocorsa\plugins\`

Edit file "plugins.ini":

```
[PLUGINS]
acplugin.dll=1
```
