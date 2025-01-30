<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!--
*** Based on the Best-README-Template Repository.
*** Thank you to them for letting us spend less time making READMEs
*** And more time writing code.
-->
<h1>Beware! This code is <strong>CURRENTLY</strong> an <strong>EXACT</strong> copy of 
<a href="https://github.com/FRC-7913/Crescendo-2024-Rewrite">our old code!</a> This will be updated soon to accomodate 
team progress.</h1>

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/frc-7913/Crescendo-2024">
    <img src="images/logo.png" alt="Logo" width="100" height="100" style="border-radius: 20%;">
  </a>

<h3 align="center">2025 Deep Dive Code</h3>

  <p align="center">
    FRC Team 7913
    <br />
    <br />
    <br />
    <a href="https://www.thebluealliance.com/team/7913">The Blue Alliance</a>
    ·
    <a href="https://www.facebook.com/NRHS7913/">Facebook</a>
    ·
    <a href="https://www.thebluealliance.com/team/7913#event-results">Results</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#robot-deployment">Robot Deployment</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap - WIP</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->

## About The Project

This will return later!!!!
<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

#### Robot Code

* [WPILib](https://docs.wpilib.org/en/latest/docs/software/what-is-wpilib.html) for just about everything
* REVLib for SPARK MAXes
* Phoenix 5 and Phoenix 6 for Victors and CANcoders, respectively

#### Autonomous Pathplanning and Following

* [PathPlanner](https://pathplanner.dev/) for visually building and effortlessly executing autonomous routines

#### Other Useful Tools

* [REV Hardware Client](https://docs.revrobotics.com/rev-hardware-client) for configuring SPARK MAXes
* [Phoenix Tuner X](https://pro.docs.ctr-electronics.com/en/stable/docs/tuner/index.html) for configuring Victors and
  CANcoders
* [FRC Plugin](https://plugins.jetbrains.com/plugin/9405-frc) for JetBrains IDEs
* SSH (macOS/Linux) or PuTTy (Windows)
  for [cleaning up the RoboRIO](https://docs.google.com/document/d/1Cd01X3eviaLGgd5urtJ3XSIjzB4kMR3gBS9SVEG7tIg/) of old
  PathPlanner paths and JAR deployments

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->

## Getting Started

This project is deployed in the same way as any other WPI
Project. [Their page](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/deploying-robot-code.html) also
covers deployment.

### Prerequisites

This assumes you have a working RoboRio connected to power, with or without a transmitter.

> **Important:** You must build the robot code once while connected to the Internet to download all the dependencies.

To ensure all dependencies are downloaded, you must build the robot code before connecting to the robot WiFi. There are
three ways to do this.

1. With WPILib VS Code, click on the WPILib icon or open the command palette and type "WPILib". Select
   `Build Robot Code`.
2. In IntelliJ, run the `Build Robot` task.
3. On the command line, run `gradle build` (with Gradle installed), `gradlew.bat build` (on Windows), or
   `./gradlew build` (on macOS or Linux).

Once the code dependencies are installed, connect to the robot. There are two ways to do this.

1. By WiFi. With the robot WiFi network set up, connect your computer to the robot's WiFi network
2. By cable. Connect an ethernet cable from the robot to your computer.

### Robot Deployment

With your computer connected to the robot, run the deploy task.

In VSCode, click on the WPILib icon or open the command palette and type WPILib. Select `Deploy Robot Code`.

In IntelliJ, select the `Build & Deploy Robot` command.

On the command line, run `gradle deploy` (with Gradle installed), `gradlew.bat deploy` (on Windows), or
`./gradlew deploy` (on macOS or Linux).

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->

## Usage

Each mode has a specific purpose.

##### Teleop

Teleop will allow you to control the robot directly.

##### Autonomous

Runs the robot without driver input.

##### Test

Not yet implemented. Do not select - for now.
<!-- ROADMAP -->

## Roadmap

A roadmap will eventually be established - probably on [Notion](https://Notion.so)


<!-- CONTRIBUTING -->

## Contributing

Generally, work on this project will be done by the 7913 Programming team.
Contributions, however, are welcome if you see some glaring issue.

The easiest way to contribute is to open an issue.
You can open an issue and tag it with `bug` or `enhancement`.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->

## License

Distributed under the MIT License. See `LICENSE` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->

## Contact

Team Mentor Email - charlesestes@OEM-tech.net

Use this email for any important team communication, but for anything related to the code refer to Github issues.

Project Link: This will be back!!!!

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->

## Acknowledgments

* Our mentor, [Dillon](https://github.com/dillontherrien) for help with organization and code in the 2024 season
* Our mentor, [Mrs. Mayo](https://github.com/MrsMayo-NRHS) for help with code planning in the 2024 season

<p align="right">(<a href="#readme-top">back to top</a>)</p>
