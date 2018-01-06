This section tries to give guidance about how to improve the software quality of ROS2 packages. The guide uses a pattern language based approach to improve the readers experience ("read little, understand fast, understand much, apply easily").

**What this sections is about:**

- ROS2 core, application and ecosystem packages.
- ROS2 core client libraries C++ and Python (right now: mainly C++)
- Design and implementation considerations to improve quality attributes like "Reliability", "Security", "Maintainability", "Determinism", etc. which relate to non-functional requirements (right now: mainly "Reliability").

**What this section is not about:**

- Design and implementation considerations which go beyond a single ROS2 package and a single ROS2 node (means no integration considerations w.r.t. ROS2 graphs, etc.).
- Organizational considerations to improve software quality (an organizations structure and processes, etc.).
- Infrastructural considerations which go beyond a single repository (overall continuous integration infrastructure, etc.)

## Patterns

* Static code analysis
  * Static code analysis using a single tool
  * Static code analysis using multiple tools complementary
* Dynamic code analysis
* ROS2 library test
  * (referencing of generic unit test patterns like from [xUnitPatterns](http://xunitpatterns.com/Book%20Outline%20Diagrams.html) with references to C++ gtest+gmock/Python unittest implementations)
  * (ROS2 specific unit test use cases)
  * Property based test (C++ [RapidCheck](https://github.com/emil-e/rapidcheck) / Python [hypothesis](https://github.com/HypothesisWorks/hypothesis-python))
  * Code coverage analysis
* ROS2 node unit test
  * (generic use cases of `launch` based tests)

## Code coverage analysis

**Context**

You have written tests for the library level production code of a ROS2 package (usually refered to as "unit tests").

**Problem**

You do not know how much of the production code is exercised during the execution of the tests.

**Solution**

Select and use a code coverage analysis tool to determine the code coverage.

**Forces**

* Is it possible to integrate the tool with your source code editor?
* If not web service based: Is it possible to integrate the tool with your continuous integration infrastructure?
* What type(s) of coverage measurements (e.g. statement coverage) does the tool support?

**Example**

* C++
  * [gcov](https://gcc.gnu.org/onlinedocs/gcc/Gcov.html) + [lcov](http://ltp.sourceforge.net/coverage/lcov.php)
  * [coveralls.io](https://coveralls.io)
* Python
  * [coveralls.io](https://coveralls.io)

**Resulting context**

* You know how much of your production code was exercised during the execution of the unit tests.
* You have a more or less immediate feedback about the code coverage (editor integration / web service front end).
* You do not know anything about the quality of your tests. (The only way to figure that out is some kind of review).