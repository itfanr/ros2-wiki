# Build Cop and Build Farmer Guide

This page covers two rotating developer roles we have on the ROS 2 team, the build cop and the build farmer.
These two roles are related, but subtly different.

The build cop is responsible for monitoring our [Continuous Integration (CI) server](http://ci.ros2.org/) to make sure our ["nightly" jobs](http://ci.ros2.org/view/nightly/) and ["packaging" jobs](http://ci.ros2.org/view/packaging/) to not accumulate new regressions.
This allows us to build on relatively stable ["ci" (or "manual") jobs](http://ci.ros2.org/) when checking to see if a new feature or bug fix introduces any new regressions.

The build farmer, on the other hand, is responsible for ensuring that [the machines that run all of our CI jobs](http://ci.ros2.org/computer/) are up, running, and up-to-date, so that they are available for other developers to check their pull requests.

These two jobs have so far been the same person for periods of about two weeks, but they are separated here so that they could be different people in the future and to make a distinction between the roles and responsibilities.

This document is meant mostly for ROS 2 team developers, but it might be useful for others too.

## On-boarding

If you are becoming the next build cop or build farmer you should:

- Make sure you are subscribed to and have the "deliver every email" option turned on for this mailing list:
  - https://groups.google.com/forum/#!forum/ros2-buildfarm
- Make sure you are "watching" this GitHub repository:
  - https://github.com/ros2/build_cop
- Talk with the previous build cop or build farmer about on-going issues

## Retirement

If you are finishing your stint as either build cop or build farmer you should:

- Unsubscribe or switch to "digest" for this mailing list:
  - https://groups.google.com/forum/#!forum/ros2-buildfarm
- "Unwatch" this GitHub repository:
  - https://github.com/ros2/build_cop
- Summarize the on-going issues for the next build cop or build farmer to which you are handing off

## Build Cop

This section assumes that you have reviewed the [ROS 2 on boarding document](https://github.com/ros2/ros2/wiki/ROS-2-On-boarding-Guide).

### Mission

The goal of the build cop is to keep the jobs "green" (succeeding without test failures or warnings) on the buildfarm and to report any regression to the appropriate person.
This will reduce the overhead of several people looking at, or investigating, the same build failures.
This will also allow the other people on the team not to receive nightly email for the failing and/or unstable builds.

### Build Cop Tasks

Every morning the build cop should go through all the nightly jobs and packaging jobs and act on new failing or unstable jobs.
The "ci" jobs, which are started manually by developers, are not the responsibility of the build cop unless all "ci" are failing, which would indicate something was merged to the default branches which is broken.
How to classify and deal with new failures is described in the next section.

Additionally, the build cop should strive to keep track of existing issues which are either preexisting or cannot be resolved with a day or so.
Issues are tracked on this repository which only the build cop must be "watching" (getting GitHub notifications), though anyone interested can follow it too:

https://github.com/ros2/build_cop/issues

The above repository is also used to track Build Farmer issues.
It is meant to be a way for the build cop or build farmer to track long running items for handing off to the next person and so they can "mention" individuals on particular issues without the whole team getting notified of every issue.

#### Types of Failures

Each failure can be categorized into one of a few classifications:

- Node failure:
  - failures which appear to be due to a machine configuration and not a code change
  - pass it on to the build farmer
- Trivial failure:
  - Linter failure
  - New warnings
- Critical failure:
  - Breaking builds on the "default" branch
  - Regression (existing tests which were passing are now failing)
  - New tests that are failing (never were passing)
- Important failure:
  - New feature which is not fully covered by tests
  - Increase flakiness (new flaky tests or made existing flaky tests more flaky)

#### New Failure Actions

In each case different actions should be taken by the build cop:

- Trivial failure should be fixed by the Build Cop right away
  - Pushed on a branch
  - Tested on CI
  - Merged to the default branch
  - Add a comment to the PR introducing the failure referencing the fixing commit
- Critical failures:
  - Failure should be reported on the PR introducing it by tagging the submitter and the reviewer that +1’d it.
  - Submitter of the PR has to act on it during the next half day by either:
    - Reverting the change and ticketing the problem / comment on the PR the reason it’s been reverted
    - Submit a patch to fix the failure
- Important failures:
  - Failure should be reported on the PR introducing it by tagging the submitter + the reviewer the +1’d it.
  - Submitter has to either:
    - Address it the same week
    - Add it to the next sprint

#### Tips

- You can search for PRs merged between two dates with, e.g.:
  - https://github.com/search?utf8=%E2%9C%93&q=user%3Aament+user%3Aros2+merged%3A%222017-04-17T22%3A00%3A00-08%3A00+..+2017-04-18T23%3A30%3A00-08%3A00%22&type=Issues
  - (note that FastRTPS and other external repos won’t appear)
- Times displayed at the top of jenkins jobs are in UTC. You can convert the times with e.g.:
  - https://www.google.com/webhp?sourceid=chrome-instant&ion=1&espv=2&ie=UTF-8#q=when+it's+9:25:12+PM+UTC+time+pst

## Build Farmer

This section assumes that you have reviewed the [ROS 2 on boarding document](https://github.com/ros2/ros2/wiki/ROS-2-On-boarding-Guide).

### Mission

The mission of the build farmer is to keep the [build farm](http://ci.ros2.org/) in a healthy, up-to-date state.
Ideally all Jenkins nodes will have the same/latest version of every package.
This will require monitoring and patching up any node when things come up.

### Build Farmer Tasks

- Monitor the buildfarm (using the https://groups.google.com/forum/#!forum/ros2-buildfarm mailing list for email notifications)
- Take failing nodes off-line with descriptive message about the issue and investigate the failure ASAP.
  - Taking a node off-line:
    - log in ci.ros2.org
    - click on the node in the left column of Jenkins UI (e.g. osx_slave_mini1)
    - click on "Mark this node temporary off-line" button (top right corner)
  - Investigate the failure (see section below)
- Report error and fixing attempt using the [Build Farmer Reporting Form](https://docs.google.com/a/osrfoundation.org/forms/d/e/1FAIpQLSc40KMD8hb1-JMkUBRF6o17CAt1mtEQY8w4O8PN8rFq0hEkxQ/viewform)
  - If the same problem and action is taken on multiple nodes, select all the relevant nodes in the form before submitting it
- If fixing attempt failed:
  - Keep the node off-line on the farm
  - Update the reason for node being off-line
  - Put the status and error message in the status sheet of the [logbook](https://docs.google.com/a/osrfoundation.org/spreadsheets/d/1_7pv1Kb2MDhk4jzpS1cjTVJ6wgdVIN3ZxaN-QdZ7XuM/edit?usp=sharing)
  - Allocate time in the next few days to dive in the problem
- If you rescue a previously off-line node:
  - Update the status sheet of the logbook by:
    - Removing the error message in the status column
    - Update the date next to it
- Once investigation is finished: clean the machine: close all your windows, stash or remove any local changes
  - Rationale
    - Anybody logging into the machine need to know that no one is working on it
    - If the machine reboots the machine needs to be in an operational state without local changes
- Making sure install instructions are up to date
- Use the existing logbook to put together an FAQ or best practice to rescue nodes

### How to Investigate a Failing Node

If the node failed to build: look at the console output of the failing build;

- Click on the link of the failing job received by email or go to ci.ros2.org and click on the number (e.g. #2345) next to the failing job(red icon).
- Click on Console Output in the left column
- Look for the reason of the failure
- If the failure is not due to a machine configuration, relay to the build cop

If you need to access a machine:

- See this spreadsheet (private) for credentials for all the different Jenkins Nodes:
  - https://docs.google.com/a/osrfoundation.org/spreadsheets/d/1OSwqbE3qPF8v3HSMr8JOaJ6r4QOiQFk6pwgaudXVE-4/edit?usp=sharing
- For machines hosted at OSRF, you'll need to be on the OSRF network or have a VPN connection.
- For machines which require <ssh keys> ask on ros@osrfoundation.org for your public keys to be added.
- The Packet.net nodes (with "packet" in the name):
  - Can be managed by logging in at https://packet.net with <brian’s email address>/<the usual company password>
    - This will change after we set up a team account to manage servers.
  - Can be accessed with Tully’s or Brian’s ssh key.
    - This will change after we set up a team account to manage servers.

### Troubleshooting

If a node goes off-line:

- For machines with VNC, you should try that first because many failures can be due to pop-up windows or required updates
- If you don’t have any pop-ups and relaunching the jenkins client doesn’t fix it, then you'll have to start troubleshooting.
- Looking at configuration difference between the nodes may be useful (java version, pip freeze, etc.)
- For Linux nodes that have gone off-line (e.g. because of a reboot), they can be reconnected through the Jenkins web interface of that node

Other tips:

----

The environment variables on Windows machines are output at the beginning of Jenkins jobs (search for `==> set`).
If you are modifying environment variables on Windows nodes, you may need to restart the machine before the changes are reflected in the jobs.
This is due to the Jenkins slave session caching the environment variables to some degree.

----

On the Windows machines, the Jenkins slave program runs as a service as the System account.
For this user, the "home" directory seems to be `C:\Windows\system32\config\systemprofile`.
You can "become" the system user to debug stuff by downloading `pxexec`:

https://technet.microsoft.com/en-us/sysinternals/pxexec

Then you extract the zip, then open a command-prompt as administrator, and then run: `psexec -i -s cmd.exe`

This is all pieced together from a couple of pages here:

http://blog.thomasvandoren.com/jenkins-windows-slave-with-git.html

and here:

https://answers.atlassian.com/questions/128324/where-is-the-home-directory-for-the-system-user

----

Every so often the router reboots. The mac machines usually don’t reconnect to Jenkins properly. Just manually reconnect them.

### Resources

- How to setup the Jenkins master:
  - https://github.com/ros2/ros2/wiki/CI-Server-Setup
- How to setup Linux Jenkins nodes:
  - https://github.com/ros2/ros2/wiki/Set-up-a-new-Linux-CI-node
- How to setup a macOS Jenkins node:
  - https://docs.google.com/a/osrfoundation.org/document/d/1J_8O7Q7eiixC-axyjP_bVpZSALyhn67Y1K_-SAw5eh0/edit?usp=sharing
  - TODO make this a wiki page
- How to setup a Windows Jenkins node:
  - https://docs.google.com/a/osrfoundation.org/document/d/1SmmWa7MVnwjmMw9XJF33-fsa0dtkYj2AeEXBa8BCsYs/edit?usp=sharing
  - TODO make this a wiki page
