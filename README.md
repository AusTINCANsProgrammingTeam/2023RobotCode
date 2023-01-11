# 2023 Robot Code 

### Welcome to FRC Team 2158 austinCANs' robot code repo for the 2023 FRC season!

#### For students/contributors:
- Follow the [WPILib Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) to setup your programming environment
- New to Git? Try out  interactive tutorial: https://learngitbranching.js.org/
- Read over (or at least look at the pictures) of this Git branching strategy:  https://nvie.com/posts/a-successful-git-branching-model/
	- TL:DR **develop** is our working base branch that feature branches will spawn off of and merge back to.  **Main** is our production branch, and release and hotfix branches are used as intermidiate branches to manage bug fixes before pushing to **main**.
- [Git Extensions](https://gitextensions.github.io/) is our recommended Git client

##### Pull Requrest (PR) policies
- Simulation of feature required before starting PR from feature branch to **develop**, 
	- On-robot testing will be required before PR is completed if hardware is available to test on.
- Always delete feature branches after a PR is completed, make a new feature branch to start a new feature.
- Release/Hotfix branches must pass simulation and full system on-robot testing to finish PR into **main**. 
- Any update to **main** should create a new production version tag.
