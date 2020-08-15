# ac-plugin

C++ SDK for Assetto Corsa.

Provides full access to game internals.

Shows how to implement custom plugins and extend/replace physics.

Requires VS2013 community, v120 platform toolset.

Don't work with recent update, use build 2426371 (acs.exe -> MD5 036e7f48676d3f43856aa37e21629708).

## Installing

Build & Copy acplugin.dll to game dir:

`d:\steam\steamapps\common\assettocorsa\plugins\`

Edit file "plugins.ini":

```
[PLUGINS]
acplugin.dll=1
```

Custom physics implemented as proxy dinput8.dll, just copy to game dir.
