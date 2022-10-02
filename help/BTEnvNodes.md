# BT Nodes for Environment variables
## Description

Write environment variable {var} to blackboard variable {XXX_out}
XXX needs to match the extracted type.

## Returns

SUCCESS if env variable has been been written to blackboard

FAILURE if no such env variable exists

## Examples:

```
<GetEnvString var="OM_AUTOMATIC_START" str_out="{strenvtest}"/>
<SayString message="{strenvtest}"/>
```

```
<GetEnvInt var="OM_OUTLINE_COUNT" int_out="{intenvtest}"/>
<SayInt message="{intenvtest}"/>
```

```
<GetEnvBoolAsInt var="OM_ENABLE_MOWER" int_out="{intenvtest}"/>
<SayInt message="{intenvtest}"/>
```

```
<GetEnvFloat var="OM_DOCKING_DISTANCE" int_out="{floatenvtest}"/>
<SayInt message="{floatenvtest}"/>
```
