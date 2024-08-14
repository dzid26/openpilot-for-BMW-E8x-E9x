<div align="center" style="text-align: center;">

<h1>openpilot</h1>

<p>
  <b>openpilot is an operating system for robotics.</b>
  <br>
  Currently, it upgrades the driver assistance system in 275+ supported cars.
</p>

<h3>
  <a href="https://docs.comma.ai">Docs</a>
  <span> · </span>
  <a href="https://docs.comma.ai/contributing/roadmap/">Roadmap</a>
  <span> · </span>
  <a href="https://github.com/commaai/openpilot/blob/master/docs/CONTRIBUTING.md">Contribute</a>
  <span> · </span>
  <a href="https://discord.comma.ai">Community</a>
  <span> · </span>
  <a href="https://comma.ai/shop">Try it on a comma 3X</a>
</h3>

Quick start: `bash <(curl -fsSL openpilot.comma.ai)`

![openpilot tests](https://github.com/commaai/openpilot/actions/workflows/selfdrive_tests.yaml/badge.svg)
[![codecov](https://codecov.io/gh/commaai/openpilot/branch/master/graph/badge.svg)](https://codecov.io/gh/commaai/openpilot)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![X Follow](https://img.shields.io/twitter/follow/comma_ai)](https://x.com/comma_ai)
[![Discord](https://img.shields.io/discord/469524606043160576)](https://discord.comma.ai)

---
BMW integration
------
Installer link: https://installer.comma.ai/dzid26/master
This is development fork for openpilot integration specific to BMW E-series 2008-2013. (BMW E8x, E9x, maybe E60).
(My car is E82 with added DCC and servotronic).

- **Lateral control** is optional and supported via external streering actuator based on [STEPPER_SERVO_CAN](https://github.com/dzid26/StepperServoCAN).
- **Longitudinal control** is supported via injecting cruise control +/- stalk commands.
Since it uses cruise control CAN bus requests to adjust speed it inherits limitation of the stock cruise control, such as minimum 20mph engagement speed and acceelration rates.
The speed target relies purely on openpilot vision model and it may not always be accurate too.

BMW E-series 2008-2013 have 3 vehicle options regarding the cruise control:
- VO540 **[supported]** - normal cruise control - managed by ECU and typical for 1-series, but can be relatively easily programmed to VO544 on 6-cylinder cars[using NCSExpert](https://www.1addicts.com/forums/showthread.php?t=1138536)
- VO544 **[supported]**  - dynamic cruise control (DCC) - managed by DSC and enables brake activation if speed target is far from actual speed. This option is often present in  3-series and all E92.
- VO541 **[not supported]**  - active cruise control (ACC) - BMW's radar solution - very rare option and reportedly, it had issues with going out of calibration. It probably would work with openpilot, but was not validated. Maybe it has different minimum speed limit?

Each cruise control option can operate in mph or kph (can be changed globally via vehicle setting or only for cruise using NCSExpert). Currently control units are [hardcoded](https://github.com/dzid26/openpilot-for-BMW-E8x-E9x/blob/master-ci/selfdrive/car/bmw/carstate.py#L110) to metric for better control resolution. TODO autodetection of cruise units.

**In order for openpilot to be able to control vehicle speed, it needs to access ignition status, PT-CAN and F-CAN.** (F-CAN doesn't need to be connected with VO540)

Connection requirement:
| Car | **F_Can** | **PT_Can** |
| ---- | -------- | ---------- |
| E8x with DCC | Y | Y |
| E8x without DCC | o | Y |
| E9x with DCC| Y | Y |

Refer to [opendbc-BMW](https://github.com/dzid26/opendbc-BMW-E8x-E9x) to explore and contribute to decoding of BMW CAN messages.


DIY hardware onnections:

| Wire      |  **Main** & secondary color | Harness pin | Description                | Acceess location code |
| --------- | ------------------          | ---------  | ------------------          |------       |
| PT_CAN_H  | **blue** &  red             | CAN0H      |powertrain CAN               |   X10548    |
| PT_CAN_L  | **red**                     | CAN0L      |powertrain CAN               |   X10549    |
| F_CAN_H   |  **white** & yellow         | CAN1H      |chasis CAN (see table above) |   X13722    |
| F_CAN_L   |  **white** & blue           | CAN1L      |chasis CAN                   |   X13723    |
| Optional: |                             |            |                             |             |
| KL_15     |  **green** & red            | IGN        |ignition indicator terminal  |   X10550    |
| K_CAN_H   |  **orange** & green         | CAN2H      |body CAN (optional)          |   X15003    |
| K_CAN_L   |  **green**                  | CAN2L      |body CAN (optional)          |   X15004    |
| D_CAN_H   |  **white** & yellow         | CAN3H (obd)|diagnostic bus (optional)    | OBD2 pin 6  |
| D_CAN_L   |  **white** & blue           | CAN3L (obd)|diagnostic bus (optional)    | OBD2 pin 14 |

STEPPER_SERVO_CAN can be connected instead of K-CAN or added to F-CAN network after desoldering 120 ohm termination resistor from the PCB.

[PT-CAN](https://www.newtis.info/tisv2/a/en/e90-335i-lim/components-connectors/plug-in-comb-type-solder-connectors/connectors-from-x8/x8091-x8091/Ck5ibwF8) and Ignition status are available as splices exposed within [big wire-loom](https://www.newtis.info/tisv2/a/en/e90-325i-lim/components-connectors/plug-in-comb-type-solder-connectors/connectors-from-x1/x10550-x10550/SQCw5q4) in a corner above foot-rest plastic.
[F-CAN](https://www.newtis.info/tisv2/a/en/e90-335i-lim/components-connectors/plug-in-comb-type-solder-connectors/connectors-from-x1/x14024-x14024/B5OUNoSj) is available as splices exposed in a [wire-loom](https://www.e90post.com/forums/showpost.php?p=20414970&postcount=9) below front of the driver door frame (under foot-well floor lining).
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

openpilot is developed by [comma](https://comma.ai/) and by users like you. We welcome both pull requests and issues on [GitHub](http://github.com/commaai/openpilot).

* Join the [community Discord](https://discord.comma.ai)
* Check out [the contributing docs](docs/CONTRIBUTING.md)
* Check out the [openpilot tools](tools/)
* Read about the [development workflow](docs/WORKFLOW.md)
* Code documentation lives at https://docs.comma.ai
* Information about running openpilot lives on the [community wiki](https://github.com/commaai/openpilot/wiki)

Want to get paid to work on openpilot? [comma is hiring](https://comma.ai/jobs#open-positions) and offers lots of [bounties](https://comma.ai/bounties) for external contributors.

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
