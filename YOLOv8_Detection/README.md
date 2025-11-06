# YOLOv8_Detection

> **📢 ROS1 Integration Available!**  
> This component now supports ROS1! See [ROS1_README.md](ROS1_README.md) for details on using YOLOv8_Detection as a ROS1 node.

## Overview

Detect object using YOLOv8 ONNX model

This component is available in two variants:
- **OpenRTM-aist component** (original) - Use `CMakeLists_openrtm.txt`
- **ROS1 node** (new) - Use `CMakeLists.txt` with catkin build system

## Description



### Input and Output



### Algorithm etc



### Basic Information

|  |  |
----|---- 
| Module Name | YOLOv8_Detection |
| Description | Detect object using YOLOv8 ONNX model |
| Version | 1.0.0 |
| Vendor | ha-ves |
| Category | ImageProcessiong |
| Comp. Type | COMMUTATIVE |
| Act. Type | SPORADIC |
| Kind | DataFlowComponent |
| MAX Inst. | 0 |

### Activity definition

<table>
  <tr>
    <td rowspan="4">on_initialize</td>
    <td colspan="2">implemented</td>
    <tr>
      <td>Description</td>
      <td></td>
    </tr>
    <tr>
      <td>PreCondition</td>
      <td></td>
    </tr>
    <tr>
      <td>PostCondition</td>
      <td></td>
    </tr>
  </tr>
  <tr>
    <td>on_finalize</td>
    <td colspan="2"></td>
  </tr>
  <tr>
    <td rowspan="4">on_startup</td>
    <td colspan="2">implemented</td>
    <tr>
      <td>Description</td>
      <td></td>
    </tr>
    <tr>
      <td>PreCondition</td>
      <td></td>
    </tr>
    <tr>
      <td>PostCondition</td>
      <td></td>
    </tr>
  </tr>
  <tr>
    <td>on_shutdown</td>
    <td colspan="2"></td>
  </tr>
  <tr>
    <td rowspan="4">on_activated</td>
    <td colspan="2">implemented</td>
    <tr>
      <td>Description</td>
      <td></td>
    </tr>
    <tr>
      <td>PreCondition</td>
      <td></td>
    </tr>
    <tr>
      <td>PostCondition</td>
      <td></td>
    </tr>
  </tr>
  <tr>
    <td rowspan="4">on_deactivated</td>
    <td colspan="2">implemented</td>
    <tr>
      <td>Description</td>
      <td></td>
    </tr>
    <tr>
      <td>PreCondition</td>
      <td></td>
    </tr>
    <tr>
      <td>PostCondition</td>
      <td></td>
    </tr>
  </tr>
  <tr>
    <td rowspan="4">on_execute</td>
    <td colspan="2">implemented</td>
    <tr>
      <td>Description</td>
      <td></td>
    </tr>
    <tr>
      <td>PreCondition</td>
      <td></td>
    </tr>
    <tr>
      <td>PostCondition</td>
      <td></td>
    </tr>
  </tr>
  <tr>
    <td>on_aborting</td>
    <td colspan="2"></td>
  </tr>
  <tr>
    <td>on_error</td>
    <td colspan="2"></td>
  </tr>
  <tr>
    <td rowspan="4">on_reset</td>
    <td colspan="2">implemented</td>
    <tr>
      <td>Description</td>
      <td></td>
    </tr>
    <tr>
      <td>PreCondition</td>
      <td></td>
    </tr>
    <tr>
      <td>PostCondition</td>
      <td></td>
    </tr>
  </tr>
  <tr>
    <td>on_state_update</td>
    <td colspan="2"></td>
  </tr>
  <tr>
    <td>on_rate_changed</td>
    <td colspan="2"></td>
  </tr>
</table>

### InPorts definition

#### in_image



<table>
  <tr>
    <td>DataType</td>
    <td>RTC::CameraImage</td>
    <td></td>
  </tr>
  <tr>
    <td>IDL file</td>
    <td colspan="2">InterfaceDataTypes.idl</td>
  </tr>
  <tr>
    <td>Number of Data</td>
    <td colspan="2"></td>
  </tr>
  <tr>
    <td>Semantics</td>
    <td colspan="2"></td>
  </tr>
  <tr>
    <td>Unit</td>
    <td colspan="2"></td>
  </tr>
  <tr>
    <td>Occirrence frecency Period</td>
    <td colspan="2"></td>
  </tr>
  <tr>
    <td>Operational frecency Period</td>
    <td colspan="2"></td>
  </tr>
</table>


### OutPorts definition

#### out_objects



<table>
  <tr>
    <td>DataType</td>
    <td>RTC::TimedUShort</td>
    <td></td>
  </tr>
  <tr>
    <td>IDL file</td>
    <td colspan="2">BasicDataType.idl</td>
  </tr>
  <tr>
    <td>Number of Data</td>
    <td colspan="2"></td>
  </tr>
  <tr>
    <td>Semantics</td>
    <td colspan="2"></td>
  </tr>
  <tr>
    <td>Unit</td>
    <td colspan="2"></td>
  </tr>
  <tr>
    <td>Occirrence frecency Period</td>
    <td colspan="2"></td>
  </tr>
  <tr>
    <td>Operational frecency Period</td>
    <td colspan="2"></td>
  </tr>
</table>


### Service Port definition


### Configuration definition

#### ONNX_File




<table>
  <tr>
    <td>DataType</td>
    <td colspan="2">string</td>
  </tr>
  <tr>
    <td>DefaultValue</td>
    <td>none</td>
    <td></td>
  </tr>
  <tr>
    <td>Unit</td>
    <td></td>
    <td></td>
  </tr>
  <tr>
    <td>Widget</td>
    <td colspan="2">text</td>
  </tr>
  <tr>
    <td>Step</td>
    <td colspan="2"></td>
  </tr>
  <tr>
    <td>Constraint</td>
    <td></td>
    <td></td>
  </tr>
  <tr>
    <td>Range</td>
    <td colspan="2"></td>
  </tr>
</table>

#### Confidence_Threshold




<table>
  <tr>
    <td>DataType</td>
    <td colspan="2">float</td>
  </tr>
  <tr>
    <td>DefaultValue</td>
    <td>0.75</td>
    <td></td>
  </tr>
  <tr>
    <td>Unit</td>
    <td>100s% (Percent)</td>
    <td></td>
  </tr>
  <tr>
    <td>Widget</td>
    <td colspan="2">slider</td>
  </tr>
  <tr>
    <td>Step</td>
    <td colspan="2">0.01</td>
  </tr>
  <tr>
    <td>Constraint</td>
    <td>0.01<=x<=1.0</td>
    <td></td>
  </tr>
  <tr>
    <td>Range</td>
    <td colspan="2"></td>
  </tr>
</table>

#### Overlap_IoU_Threshold




<table>
  <tr>
    <td>DataType</td>
    <td colspan="2">float</td>
  </tr>
  <tr>
    <td>DefaultValue</td>
    <td>0.75</td>
    <td></td>
  </tr>
  <tr>
    <td>Unit</td>
    <td>100s% (Percent)</td>
    <td></td>
  </tr>
  <tr>
    <td>Widget</td>
    <td colspan="2">slider</td>
  </tr>
  <tr>
    <td>Step</td>
    <td colspan="2">0.01</td>
  </tr>
  <tr>
    <td>Constraint</td>
    <td>0.01<=x<=1.0</td>
    <td></td>
  </tr>
  <tr>
    <td>Range</td>
    <td colspan="2"></td>
  </tr>
</table>


## Demo

## Requirement

## Setup

### Windows

### Ubuntu

## Usage

## Running the tests

## LICENCE




## References




## Author


