<div align="center" style="text-align: center;">

<h1>openpilot</h1>

<p>
  <b>openpilot is an operating system for robotics.</b>
  <br>
  Currently, it upgrades the driver assistance system in 275+ supported cars.
</p>

<h3>
  <a href="https://docs.comma.ai">Read the docs</a>
  <span> · </span>
  <a href="https://github.com/commaai/openpilot/blob/master/docs/CONTRIBUTING.md">Contribute</a>
  <span> · </span>
  <a href="https://discord.comma.ai">Community</a>
  <span> · </span>
  <a href="https://comma.ai/shop">Try it on a comma 3X</a>
</h3>

Quick start: `curl -fsSL https://github.com/dzid26/opendbc-BMW-E8x-E9x | bash`

![openpilot tests](https://github.com/commaai/openpilot/actions/workflows/selfdrive_tests.yaml/badge.svg)
[![codecov](https://codecov.io/gh/commaai/openpilot/branch/master/graph/badge.svg)](https://codecov.io/gh/commaai/openpilot)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![X Follow](https://img.shields.io/twitter/follow/comma_ai)](https://x.com/comma_ai)
[![Discord](https://img.shields.io/discord/469524606043160576)](https://discord.comma.ai)

</div>
[![](https://i.imgur.com/UelUjKAh.png)](#)


---
BMW integration
------
This is development fork for openpilot integration specific to BMW E-series 2008-2013.

Currently only **longitudinal control** is supported.  (BMW E9x, E8x, maybe E60)
It uses cruise control CANbus to request speed so it also inherits limitation of the stock cruise control, such as minimum 20mph engagement speed.
The speed target relies purely on openpilot vision model.

BMW E-series 2008-2013 have 3 vehicle options for cruise control:
- VO540 **[supported]** - normal cruise control - managed by ECU and typical for 1-series, but can be relatively easily programmed to VO544 [using NCSExpert](https://www.1addicts.com/forums/showthread.php?t=1138536)
- VO544 **[supported]**  - dynamic cruise control - managed by DSC and enables brake activation if speed target is far from actual speed. This option is often present in 3-series.
- VO541 **[not supported]**  - active cruise control - BMW's radar solution - very rare option and reportedly, it had issues with going out of calibration. It probably would work with openpilot, but was not validated. Maybe it has different minimum speed limit?

Each cruise control option can operate in mph or kph (can be changed globally via vehicle setting or only for cruise using NCSExpert). Currently control units are [hardcoded](https://github.com/dzid26/openpilot-for-BMW-E8x-E9x/blob/master-ci/selfdrive/car/bmw/carstate.py#L110) to metric for better control resolution. TODO autodetection of cruise units.

**In order for openpilot to be able to control vehicle speed, it needs to access ignition status, PT-CAN and F-CAN.** (F-CAN could be skipped for VO540, but currently control uses generalized solution with both buses).
Refer to [opendbc-BMW](https://github.com/dzid26/opendbc-BMW-E8x-E9x) to explore and contribute to decoding of BMW CAN messages.


DIY hardware onnections:

| Wire      |  **Main** & secondary color | Panda  pin | Description                 | Splice location |
| --------- | ------------------          | ---------  | ------------------          |------     |
| PT_CAN_H  | **blue** &  red             | CAN2H      |powertrain CAN               |X10548     |
| PT_CAN_L  | **red**                     | CAN2L      |powertrain CAN               |X10549     |
| F_CAN_H   |  **white** & yellow         | CAN1H      |chasis CAN                   |X13722     |
| F_CAN_L   |  **white** & blue           | CAN1L      |chasis CAN                   |X13723     |
| KL_15     |  **green** & red            | IGN        |ignition indicator terminal  |X10550     |

[PT-CAN](https://www.newtis.info/tisv2/a/en/e90-335i-lim/components-connectors/plug-in-comb-type-solder-connectors/connectors-from-x8/x8091-x8091/Ck5ibwF8) and Ignition status are available as splices exposed within [big wire-loom](https://www.newtis.info/tisv2/a/en/e90-325i-lim/components-connectors/plug-in-comb-type-solder-connectors/connectors-from-x1/x10550-x10550/SQCw5q4) in a corner above foot-rest.
[F-CAN](https://www.newtis.info/tisv2/a/en/e90-335i-lim/components-connectors/plug-in-comb-type-solder-connectors/connectors-from-x1/x14024-x14024/B5OUNoSj) is available as splices exposed in a [wire-loom](https://www.newtis.info/tisv2/a/en/e90-325i-lim/components-connectors/plug-in-comb-type-solder-connectors/connectors-from-x1/x13723-x13723/RKITgwO)  under driver foot-well floor lining.
*(This is for LHD car. Verify CANbus placement for RHD)*


What is openpilot?
------

[openpilot](http://github.com/commaai/openpilot) is an open source driver assistance system. Currently, openpilot performs the functions of Adaptive Cruise Control (ACC), Automated Lane Centering (ALC), Forward Collision Warning (FCW) and Lane Departure Warning (LDW) for a growing variety of supported [car makes, models and model years](#supported-cars). In addition, while openpilot is engaged, a camera based Driver Monitoring (DM) feature alerts distracted and asleep drivers.

<table>
  <tr>
    <td><a href="https://youtu.be/NmBfgOanCyk" title="Video By Greer Viau"><img src="https://github.com/commaai/openpilot/assets/8762862/2f7112ae-f748-4f39-b617-fabd689c3772"></a></td>
    <td><a href="https://youtu.be/VHKyqZ7t8Gw" title="Video By Logan LeGrand"><img src="https://github.com/commaai/openpilot/assets/8762862/92351544-2833-40d7-9e0b-7ef7ae37ec4c"></a></td>
    <td><a href="https://youtu.be/SUIZYzxtMQs" title="A drive to Taco Bell"><img src="https://github.com/commaai/openpilot/assets/8762862/05ceefc5-2628-439c-a9b2-89ce77dc6f63"></a></td>
  </tr>
</table>

To start using openpilot in a car
------

To use openpilot in a car, you need four things:
1. **Supported Device:** a comma 3/3X, available at [comma.ai/shop](https://comma.ai/shop/comma-3x).
2. **Software:** The setup procedure for the comma 3/3X allows users to enter a URL for custom software. Use the URL `openpilot.comma.ai` to install the release version.
3. **Supported Car:** Ensure that you have one of [the 275+ supported cars](docs/CARS.md).
4. **Car Harness:** You will also need a [car harness](https://comma.ai/shop/car-harness) to connect your comma 3/3X to your car.

We have detailed instructions for [how to install the harness and device in a car](https://comma.ai/setup). Note that it's possible to run openpilot on [other hardware](https://blog.comma.ai/self-driving-car-for-free/), although it's not plug-and-play.

To start developing openpilot
------

At the moment, openpilot supports the [EON DevKit](https://comma.ai/shop/products/eon-dashcam-devkit) and the [comma two](https://comma.ai/shop/products/comma-two-devkit). A [car harness](https://comma.ai/shop/products/car-harness) is recommended to connect the EON or comma two to the car. In the future, we'd like to support other platforms as well, like gaming PCs.

Supported Cars
------

| Make      | Model (US Market Reference)   | Supported Package | ACC              | No ACC accel below | No ALC below      |
| ----------| ------------------------------| ------------------| -----------------| -------------------| ------------------|
| Acura     | ILX 2016-18                   | AcuraWatch Plus   | openpilot        | 25mph<sup>6</sup>  | 25mph             |
| Acura     | RDX 2016-18                   | AcuraWatch Plus   | openpilot        | 25mph<sup>6</sup>  | 12mph             |
| BMW       | E8x, E9x                      | VO544, VO540      | openpilot        | 20mph<sup>6</sup>  | 20mph             |
| Chrysler  | Pacifica 2017-18              | Adaptive Cruise   | Stock            | 0mph               | 9mph              |
| Chrysler  | Pacifica Hybrid 2017-18       | Adaptive Cruise   | Stock            | 0mph               | 9mph              |
| Chrysler  | Pacifica Hybrid 2019-20       | Adaptive Cruise   | Stock            | 0mph               | 39mph             |
| Honda     | Accord 2018-19                | All               | Stock            | 0mph               | 3mph              |
| Honda     | Accord Hybrid 2018-19         | All               | Stock            | 0mph               | 3mph              |
| Honda     | Civic Hatchback 2017-19       | Honda Sensing     | Stock            | 0mph               | 12mph             |
| Honda     | Civic Sedan/Coupe 2016-18     | Honda Sensing     | openpilot        | 0mph               | 12mph             |
| Honda     | Civic Sedan/Coupe 2019        | Honda Sensing     | Stock            | 0mph               | 2mph<sup>4</sup>  |
| Honda     | CR-V 2015-16                  | Touring           | openpilot        | 25mph<sup>6</sup>  | 12mph             |
| Honda     | CR-V 2017-19                  | Honda Sensing     | Stock            | 0mph               | 12mph             |
| Honda     | CR-V Hybrid 2017-2019         | Honda Sensing     | Stock            | 0mph               | 12mph             |
| Honda     | Fit 2018-19                   | Honda Sensing     | openpilot        | 25mph<sup>6</sup>  | 12mph             |
| Honda     | Insight 2019                  | Honda Sensing     | Stock            | 0mph               | 3mph              |
| Honda     | Odyssey 2018-20               | Honda Sensing     | openpilot        | 25mph<sup>6</sup>  | 0mph              |
| Honda     | Passport 2019                 | All               | openpilot        | 25mph<sup>6</sup>  | 12mph             |
| Honda     | Pilot 2016-18                 | Honda Sensing     | openpilot        | 25mph<sup>6</sup>  | 12mph             |
| Honda     | Pilot 2019                    | All               | openpilot        | 25mph<sup>6</sup>  | 12mph             |
| Honda     | Ridgeline 2017-19             | Honda Sensing     | openpilot        | 25mph<sup>6</sup>  | 12mph             |
| Hyundai   | Elantra 2017-19<sup>1</sup>   | SCC + LKAS        | Stock            | 19mph              | 34mph             |
| Hyundai   | Genesis 2018<sup>1</sup>      | All               | Stock            | 19mph              | 34mph             |
| Hyundai   | Santa Fe 2019<sup>1</sup>     | All               | Stock            | 0mph               | 0mph              |
| Jeep      | Grand Cherokee 2016-18        | Adaptive Cruise   | Stock            | 0mph               | 9mph              |
| Jeep      | Grand Cherokee 2019           | Adaptive Cruise   | Stock            | 0mph               | 39mph             |
| Kia       | Optima 2019<sup>1</sup>       | SCC + LKAS        | Stock            | 0mph               | 0mph              |
| Kia       | Sorento 2018<sup>1</sup>      | All               | Stock            | 0mph               | 0mph              |
| Kia       | Stinger 2018<sup>1</sup>      | SCC + LKAS        | Stock            | 0mph               | 0mph              |
| Lexus     | CT Hybrid 2017-18             | All               | Stock<sup>5</sup>| 0mph               | 0mph              |
| Lexus     | ES 2019                       | All               | openpilot        | 0mph               | 0mph              |
| Lexus     | ES Hybrid 2019                | All               | openpilot        | 0mph               | 0mph              |
| Lexus     | IS 2017-2019                  | All               | Stock            | 22mph              | 0mph              |
| Lexus     | IS Hybrid 2017                | All               | Stock            | 0mph               | 0mph              |
| Lexus     | NX Hybrid 2018                | All               | Stock<sup>5</sup>| 0mph               | 0mph              |
| Lexus     | RX 2016-17                    | All               | Stock<sup>5</sup>| 0mph               | 0mph              |
| Lexus     | RX 2020                       | All               | openpilot        | 0mph               | 0mph              |
| Lexus     | RX Hybrid 2016-19             | All               | Stock<sup>5</sup>| 0mph               | 0mph              |
| Subaru    | Crosstrek 2018-19             | EyeSight          | Stock            | 0mph               | 0mph              |
| Subaru    | Impreza 2019-20               | EyeSight          | Stock            | 0mph               | 0mph              |
| Toyota    | Avalon 2016                   | TSS-P             | Stock<sup>5</sup>| 20mph<sup>6</sup>  | 0mph              |
| Toyota    | Avalon 2017-18                | All               | Stock<sup>5</sup>| 20mph<sup>6</sup>  | 0mph              |
| Toyota    | Camry 2018-19                 | All               | Stock            | 0mph<sup>2</sup>   | 0mph              |
| Toyota    | Camry Hybrid 2018-19          | All               | Stock            | 0mph<sup>2</sup>   | 0mph              |
| Toyota    | C-HR 2017-19                  | All               | Stock            | 0mph               | 0mph              |
| Toyota    | C-HR Hybrid 2017-19           | All               | Stock            | 0mph               | 0mph              |
| Toyota    | Corolla 2017-19               | All               | Stock<sup>5</sup>| 20mph<sup>6</sup>  | 0mph              |
| Toyota    | Corolla 2020                  | All               | openpilot        | 0mph               | 0mph              |
| Toyota    | Corolla Hatchback 2019-20     | All               | openpilot        | 0mph               | 0mph              |
| Toyota    | Corolla Hybrid 2020           | All               | openpilot        | 0mph               | 0mph              |
| Toyota    | Highlander 2017-19            | All               | Stock<sup>5</sup>| 0mph               | 0mph              |
| Toyota    | Highlander Hybrid 2017-19     | All               | Stock<sup>5</sup>| 0mph               | 0mph              |
| Toyota    | Highlander 2020               | All               | openpilot        | 0mph               | 0mph              |
| Toyota    | Prius 2016                    | TSS-P             | Stock<sup>5</sup>| 0mph               | 0mph              |
| Toyota    | Prius 2017-19                 | All               | Stock<sup>5</sup>| 0mph               | 0mph              |
| Toyota    | Prius Prime 2017-20           | All               | Stock<sup>5</sup>| 0mph               | 0mph              |
| Toyota    | Rav4 2016                     | TSS-P             | Stock<sup>5</sup>| 20mph<sup>6</sup>  | 0mph              |
| Toyota    | Rav4 2017-18                  | All               | Stock<sup>5</sup>| 20mph<sup>6</sup>  | 0mph              |
| Toyota    | Rav4 2019                     | All               | openpilot        | 0mph               | 0mph              |
| Toyota    | Rav4 Hybrid 2016              | TSS-P             | Stock<sup>5</sup>| 0mph               | 0mph              |
| Toyota    | Rav4 Hybrid 2017-18           | All               | Stock<sup>5</sup>| 0mph               | 0mph              |
| Toyota    | Rav4 Hybrid 2019-20           | All               | openpilot        | 0mph               | 0mph              |
| Toyota    | Sienna 2018                   | All               | Stock<sup>5</sup>| 0mph               | 0mph              |
| Volkswagen| Golf 2016-19<sup>3</sup>      | Driver Assistance | Stock            | 0mph               | 0mph              |

<sup>1</sup>Requires a [panda](https://comma.ai/shop/products/panda-obd-ii-dongle) and open sourced [Hyundai giraffe](https://github.com/commaai/neo/tree/master/giraffe/hyundai), designed for the 2019 Sante Fe; pinout may differ for other Hyundai and Kia models. <br />
<sup>2</sup>28mph for Camry 4CYL L, 4CYL LE and 4CYL SE which don't have Full-Speed Range Dynamic Radar Cruise Control. <br />
<sup>3</sup>Requires a [custom connector](https://community.comma.ai/wiki/index.php/Volkswagen#Integration_at_R242_Camera) for the [car harness](https://comma.ai/shop/products/car-harness) <br />
<sup>4</sup>2019 Honda Civic 1.6L Diesel Sedan does not have ALC below 12mph. <br />

Community Maintained Cars and Features
------

| Make      | Model (US Market Reference)   | Supported Package | ACC              | No ACC accel below | No ALC below |
| ----------| ------------------------------| ------------------| -----------------| -------------------| -------------|
| Buick     | Regal 2018<sup>7</sup>        | Adaptive Cruise   | openpilot        | 0mph               | 7mph         |
| Cadillac  | ATS 2018<sup>7</sup>          | Adaptive Cruise   | openpilot        | 0mph               | 7mph         |
| Chevrolet | Malibu 2017<sup>7</sup>       | Adaptive Cruise   | openpilot        | 0mph               | 7mph         |
| Chevrolet | Volt 2017-18<sup>7</sup>      | Adaptive Cruise   | openpilot        | 0mph               | 7mph         |
| GMC       | Acadia Denali 2018<sup>7</sup>| Adaptive Cruise   | openpilot        | 0mph               | 7mph         |
| Holden    | Astra 2017<sup>7</sup>        | Adaptive Cruise   | openpilot        | 0mph               | 7mph         |

<sup>5</sup>When disconnecting the Driver Support Unit (DSU), openpilot ACC will replace stock ACC. For DSU locations, see [Toyota Wiki page](https://community.comma.ai/wiki/index.php/Toyota). ***NOTE: disconnecting the DSU disables Automatic Emergency Braking (AEB).*** <br />
<sup>6</sup>[Comma Pedal](https://community.comma.ai/wiki/index.php/Comma_Pedal) is used to provide stop-and-go capability to some of the openpilot-supported cars that don't currently support stop-and-go. Here is how to [build a Comma Pedal](https://medium.com/@jfrux/comma-pedal-building-with-macrofab-6328bea791e8). ***NOTE: The Comma Pedal is not officially supported by [comma](https://comma.ai).*** <br />
<sup>7</sup>Requires a [panda](https://comma.ai/shop/products/panda-obd-ii-dongle) and [community built giraffe](https://zoneos.com/volt/). ***NOTE: disconnecting the ASCM disables Automatic Emergency Braking (AEB).*** <br />

Community Maintained Cars and Features are not verified by comma to meet our [safety model](SAFETY.md). Be extra cautious using them. They are only available after enabling the toggle in `Settings->Developer->Enable Community Features`.

Installation Instructions
------

Install openpilot on a EON by entering ``https://openpilot.comma.ai`` during the installer setup.

Follow this [video instructions](https://youtu.be/3nlkomHathI) to properly mount the EON on the windshield. Note: openpilot features an automatic pose calibration routine and openpilot performance should not be affected by small pitch and yaw misalignments caused by imprecise EON mounting.

Before placing the device on your windshield, check the state and local laws and ordinances where you drive. Some state laws prohibit or restrict the placement of objects on the windshield of a motor vehicle.

You will be able to engage openpilot after reviewing the onboarding screens and finishing the calibration procedure.

Limitations of openpilot ALC and LDW
------

openpilot ALC and openpilot LDW do not automatically drive the vehicle or reduce the amount of attention that must be paid to operate your vehicle. The driver must always keep control of the steering wheel and be ready to correct the openpilot ALC action at all times.

While changing lanes, openpilot is not capable of looking next to you or checking your blind spot. Only nudge the wheel to initiate a lane change after you have confirmed it's safe to do so.

Many factors can impact the performance of openpilot ALC and openpilot LDW, causing them to be unable to function as intended. These include, but are not limited to:

* Poor visibility (heavy rain, snow, fog, etc.) or weather conditions that may interfere with sensor operation.
* The road facing camera is obstructed, covered or damaged by mud, ice, snow, etc.
* Obstruction caused by applying excessive paint or adhesive products (such as wraps, stickers, rubber coating, etc.) onto the vehicle.
* The EON is mounted incorrectly.
* When in sharp curves, like on-off ramps, intersections etc...; openpilot is designed to be limited in the amount of steering torque it can produce.
* In the presence of restricted lanes or construction zones.
* When driving on highly banked roads or in presence of strong cross-wind.
* Extremely hot or cold temperatures.
* Bright light (due to oncoming headlights, direct sunlight, etc.).
* Driving on hills, narrow, or winding roads.

The list above does not represent an exhaustive list of situations that may interfere with proper operation of openpilot components. It is the driver's responsibility to be in control of the vehicle at all times.

Limitations of openpilot ACC and FCW
------

openpilot ACC and openpilot FCW are not systems that allow careless or inattentive driving. It is still necessary for the driver to pay close attention to the vehicle’s surroundings and to be ready to re-take control of the gas and the brake at all times.

Many factors can impact the performance of openpilot ACC and openpilot FCW, causing them to be unable to function as intended. These include, but are not limited to:

* Poor visibility (heavy rain, snow, fog, etc.) or weather conditions that may interfere with sensor operation.
* The road facing camera or radar are obstructed, covered, or damaged by mud, ice, snow, etc.
* Obstruction caused by applying excessive paint or adhesive products (such as wraps, stickers, rubber coating, etc.) onto the vehicle.
* The EON is mounted incorrectly.
* Approaching a toll booth, a bridge or a large metal plate.
* When driving on roads with pedestrians, cyclists, etc...
* In presence of traffic signs or stop lights, which are not detected by openpilot at this time.
* When the posted speed limit is below the user selected set speed. openpilot does not detect speed limits at this time.
* In presence of vehicles in the same lane that are not moving.
* When abrupt braking maneuvers are required. openpilot is designed to be limited in the amount of deceleration and acceleration that it can produce.
* When surrounding vehicles perform close cut-ins from neighbor lanes.
* Driving on hills, narrow, or winding roads.
* Extremely hot or cold temperatures.
* Bright light (due to oncoming headlights, direct sunlight, etc.).
* Interference from other equipment that generates radar waves.

The list above does not represent an exhaustive list of situations that may interfere with proper operation of openpilot components. It is the driver's responsibility to be in control of the vehicle at all times.

Limitations of openpilot DM
------

openpilot DM should not be considered an exact measurements of the status of alertness of the driver.

Many factors can impact the performance of openpilot DM, causing it to be unable to function as intended. These include, but are not limited to:

* Low light conditions, such as driving at night or in dark tunnels.
* Bright light (due to oncoming headlights, direct sunlight, etc.).
* The driver face is partially or completely outside field of view of the driver facing camera.
* Right hand driving vehicles.
* The driver facing camera is obstructed, covered, or damaged.

The list above does not represent an exhaustive list of situations that may interfere with proper operation of openpilot components. A driver should not rely on openpilot DM to assess their level of attention.

User Data and comma Account
------

By default, openpilot uploads the driving data to our servers. You can also access your data by pairing with the comma connect app ([iOS](https://apps.apple.com/us/app/comma-connect/id1456551889), [Android](https://play.google.com/store/apps/details?id=ai.comma.connect&hl=en_US)). We use your data to train better models and improve openpilot for everyone.

openpilot is open source software: the user is free to disable data collection if they wish to do so.

openpilot logs the road facing camera, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver facing camera is only logged if you explicitly opt-in in settings. The microphone is not recorded.

By using openpilot, you agree to [our Privacy Policy](https://my.comma.ai/privacy). You understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.

Safety and Testing
----

* openpilot observes [ISO26262](https://en.wikipedia.org/wiki/ISO_26262) guidelines, see [SAFETY.md](docs/SAFETY.md) for more details.
* openpilot has software-in-the-loop [tests](.github/workflows/selfdrive_tests.yaml) that run on every commit.
* The code enforcing the safety model lives in panda and is written in C, see [code rigor](https://github.com/commaai/panda#code-rigor) for more details.
* panda has software-in-the-loop [safety tests](https://github.com/commaai/panda/tree/master/tests/safety).
* Internally, we have a hardware-in-the-loop Jenkins test suite that builds and unit tests the various processes.
* panda has additional hardware-in-the-loop [tests](https://github.com/commaai/panda/blob/master/Jenkinsfile).
* We run the latest openpilot in a testing closet containing 10 comma devices continuously replaying routes.

Licensing
------

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

User Data and comma Account
------

By default, openpilot uploads the driving data to our servers. You can also access your data through [comma connect](https://connect.comma.ai/). We use your data to train better models and improve openpilot for everyone.

openpilot is open source software: the user is free to disable data collection if they wish to do so.

openpilot logs the road-facing cameras, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver-facing camera is only logged if you explicitly opt-in in settings. The microphone is not recorded.

By using openpilot, you agree to [our Privacy Policy](https://comma.ai/privacy). You understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.
