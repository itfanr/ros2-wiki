# ROS 2 Developer Guide

This page defines the practices and policies we employ when developing ROS 2.


##### Table of Contents
- [General Practices](#general-practices)
- [Language Versions and Code Format](#language-versions-and-code-format)
  - [C](#c)
  - [C++](#c++)
  - [Python](#python)
  - [CMake](#cmake)
  - [Markdown](#markdown)
- [Testing](#testing)
- [Versioning](#versioning)
- [Documentation](#documentation)
  - [Filesystem layout](#filesystem-layout)


## General Practices

Some practices are common to all ROS 2 development:


### Issues

When filling an issue please make sure to:

* Include enough information for another person to understand the issue.
* In case of a bug consider to provide a [short, self contained, correct (compilable), example](http://sscce.org/).


### Pull requests

* A pull request should only focus on one change.
  Separate changes should go into separate pull requests.
* A patch should be minimal in size and avoid any kind of unnecessary changes.
* Always run CI jobs for all platforms for every pull request and include links to jobs in the pull request.
  (If you don't have access to the Jenkins job someone will trigger the jobs for you.)
* Before merging a pull request all changes should be squashed into a small number semantic commits to keep the history clear.


### Development Process

* The default branch (in most cases the master branch) must always build, pass all tests and compile without warnings.
  If at any time there is a regression it is the top priority to restore at least the previous state.
* Always build with tests enabled.
* Always run tests locally after changes and before proposing them in a pull request.
  Beside using automated tests also run the modified code path manually to ensure that the patch works as intended.
* Always run CI jobs for all platforms for every pull request and include links to jobs the pull request.


### Programming conventions

* Defensive programming: ensure that assumptions are held as early as possible.
  E.g. check every return code and make sure to least throw an exception until the case is handled more gracefully.
* All error messages must be directed to `stderr`.
* Avoid using direct streaming (`<<`) to `stdout` / `stderr` to prevent interleaving between multiple threads.
* Declare variables in the narrowest scope possible.
* Keep group of items (dependencies, imports, includes, etc.) ordered alphabetically.


## Language Versions and Code Format

In order to achieve a consistent looking product we will all follow externally (if possible) defined style guidelines for each language.
For other things like package layout or documentation layout we will need to come up with our own guidelines, drawing on current, popular styles in use now.

Additionally, where ever possible, developers should use integrated tools to allow them to check that these guidelines are followed in their editors.
For example, everyone should have a PEP8 checker built into their editor to cut down on review iterations related to style.

Also where possible, packages should check style as part of their unit tests to help with the automated detection of style issues (see [ament_lint_auto](https://github.com/ament/ament_lint/blob/master/ament_lint_auto/doc/index.rst)).


### C

We will target C99.

We will use Python's PEP7 for our C style guide, with some modifications and additions:

http://legacy.python.org/dev/peps/pep-0007/

Some modifications and additions:

- We will target C99, as we do not need to support C89 (as PEP7 recommends)
 - rationale: among other things it allows us to use both `//` and `/* */` style comments
 - rationale: C99 is pretty much ubiquitous now
- C++ style `//` comments are allowed
- Always place literals on the left hand side of comparison operators, e.g. `0 == ret` instead of `ret == 0`
 - rationale: `ret == 0` too easily turns into `ret = 0` by accident

All of the following modifications only apply if we are not writing Python modules:

- Do not use `Py_` as a prefix for everything
 - Instead use a CamelCase version of the package name or other appropriate prefix
- The stuff about documentation strings doesn't apply

We can use the `pep7` python module for style checking:

https://github.com/mike-perdide/pep7

The editor integration seems slim, we may need to looking to automated checking for C in more detail.


### C++

We will target C++11, using new built-in C++11 features over Boost equivalents where ever possible.

We will use the Google C++ Style Guide, with some modifications:

http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml

- Access Control: drop requirement for all class members to be private and therefore require accessors
 - rationale: this is overly constraining for user API design
 - we should prefer private members, only making them public when they are needed
 - we should consider using accessors before choosing to allow direct member access
 - we should have a good reason for allowing direct member access, other than because it is convenient for us
- Exceptions are allowed
 - rationale: this is a new code base, so the legacy argument doesn't apply to us
 - rationale: for user facing API's it is more idiomatic C++ to have exceptions
 - Exceptions in destructors should be explicitly avoided
- We should consider avoiding Exceptions if we intend to wrap the resulting API in C
 - rationale: it will make it easier to wrap in C
 - rationale: most of our dependencies in code we intend to wrap in C do not use exceptions anyways
- No restrictions on Lambda's or `std::function` or `std::bind`
- Boost should be avoided until absolutely required
- Use `/* */` comments for _documentation_ purposes and `//` style comments for notes and general comments
 - Class and Function comments should use `/* */` style comments
 - rationale: `/* */` style comments are recommended for Doxygen and Sphinx in C/C++
 - rationale: mixing `/* */` and `//` is convenient for block commenting out code which contains comments
 - Descriptions of how the code works or notes within classes and functions should use `//` style comments
- Prefer `char * c;` to `char* c;` or `char *c;` because of this scenario `char* c, *d, *e;`
- Do not put 1 space before `public:`, `private:`, or `protected:`, it is more consistent for all indentions to be a multiple of 2
 - rationale: most editors don't like indentions which are not a multiple of the (soft) tab size
 - Use zero spaces before `public:`, `private:`, or `protected:`, or 2 spaces
 - If you use 2 spaces before, indent other class statements by 2 additional spaces
 - Prefer zero spaces, i.e. `public:`, `private:`, or `protected:` in the same column as the class
- Never add whitespace to nested templates
 - Prefer `set<list<string>>` (C++11 feature) to `set<list<string> >` or `set< list<string> >`
- Use open braces for `function`, `class`, and `struct` definitions, but allow cuddle braces on `if`, `else`, `while`, `for`, etc...
 - Open braces are allowed with all blocks and encouraged if the statement before the opening brace is multiline

This is OK:

```c++
int main(int argc, char **argv)
{
  if (condition) {
    return 0;
  } else {
    return 1;
  }
}

if (this &&
    that ||
    both)
{
  ...
}
```

Bad:

```c++
int main(int argc, char **argv) {
  return 0;
}

if (this &&
    that ||
    both) {
  ...
}
```

- Use open braces rather than excessive indention, e.g. for distinguishing constructor code from constructor initializer lists

OK:

```c++
ReturnType LongClassName::ReallyReallyReallyLongFunctionName(
  Type par_name1,  // 2 space indent
  Type par_name2,
  Type par_name3)
{
  DoSomething();  // 2 space indent
  ...
}

MyClass::MyClass(int var)
: some_var_(var),
  some_other_var_(var + 1)
{
  ...
  DoSomething();
  ...
}
```

Bad, weird (the google way?):

```c++
ReturnType LongClassName::ReallyReallyReallyLongFunctionName(
    Type par_name1,  // 4 space indent
    Type par_name2,
    Type par_name3) {
  DoSomething();  // 2 space indent
  ...
}

MyClass::MyClass(int var)
    : some_var_(var),             // 4 space indent
      some_other_var_(var + 1) {  // lined up
  ...
  DoSomething();
  ...
}
```

Most of these styles and restrictions can be checked with a combination of Google's [cpplint.py](http://google-styleguide.googlecode.com/svn/trunk/cpplint/) and [clang-format](http://clang.llvm.org/docs/ClangFormat.html), though we may need to modify them slightly for our above changes.
Editor integration for `cpplint.py`:

- vim: https://github.com/funorpain/vim-cpplint
- emacs: http://google-styleguide.googlecode.com/svn/trunk/google-c-style.el
- Sublime Text: https://sublime.wbond.net/packages/SublimeLinter-cpplint

`clang-format` can be configured to use preexisting styles:

http://clang.llvm.org/docs/ClangFormatStyleOptions.html#configurable-format-style-options

And can be integrated into many existing editors:

- vim: http://clang.llvm.org/docs/ClangFormat.html#vim-integration
- emacs: http://clang.llvm.org/docs/ClangFormat.html#emacs-integration
- sublime text: https://llvm.org/svn/llvm-project/cfe/trunk/tools/clang-format/clang-format-sublime.py


### Python

We will target Python 3 for our development.

We will use the PEP8 guidelines for code format:

http://legacy.python.org/dev/peps/pep-0008/

```
PEP8 you can optionally allow lines up to 100 characters, instead of 80, if the docstrings and comments stay under 76 but for example the standard Python library still requires 80 characters (effectively 79 characters).

Should we follow this or make a allowance for longer lines?
```

Tools like the `pep8` python package or the `flake8` Python package should be used in unit-test and/or editor integration for checking Python code style.

Integration with editors:

- vim: https://github.com/nvie/vim-flake8
- emacs: http://kwbeam.com/emacs-for-python-i.html
- Sublime Text: https://sublime.wbond.net/packages/SublimeLinter-flake8


### CMake

We will target CMake 2.8.12.2.

Since there is not an existing CMake style guide we will define our own:

- Use lowercase keywords (functions and macros)
- Use empty `else()` and `end...()` commands
- No whitespace before `(`'s
- Use two spaces of indention, do not use tabs
- Prefer functions with `set(PARENT_SCOPE)` to macros
- When using macros prefix local variables with `_` or a reasonable prefix


### Markdown

The following rules to format the markdown syntax is intended to increase readability as well versioning.


- Each section title should be preceded by two empty line and succeeded by one empty line.
  - Rationale: It expedites to get an overview about the structure when screening the document.
- Each sentence must start on a new line.
 - Rationale: For longer paragraphs a single change in the beginning makes the diff unreadable since it carries forward through the whole paragraph.
- Each sentence can optionally be wrapped to keep each line short.
- The lines should not have any trailing white spaces.


### Javascript

*(Speculative, not yet used)*

We will target Javascript 1.5, which seems to provide the best balance of support in browsers and languages (node.js) and new features.

We will use the airbnb Javascript Style guide:

https://github.com/airbnb/javascript

The above repository comes with a `jshintrc` file which allows the style to be enforced using `jshint`.
Editor integration for `jshint` include `vim`, `emacs`, `Sublime Text`, and others:

http://www.jshint.com/install/


## Testing

All packages should have some level of tests.
Tests can be broken down into three main categories, System tests, Integration tests, and Unit tests.

Unit tests should always be in the package which is being tested and should make use of tools like `Mock` to try and test narrow parts of the code base in constructed scenarios.
Unit tests should not bring in test dependencies that are not testing tools, e.g. gtest, nosetest, mock, etc...

Integration tests can test interactions between parts of the code or between parts of the code and the system.
They often test software interfaces in ways that we expect the user to use them.
Like Unit tests, Integration tests should be in the package which is being tested and should not bring in non-tool test dependencies unless absolutely necessary, i.e. all non-tool dependencies should only be allowed under extreme scrutiny so they should be avoided if possible.

System tests are designed to test end-to-end situations between packages and should be in their own packages to avoid bloating or coupling packages and to avoid circular dependencies.

In general minimizing external or cross package test dependencies should be avoided to prevent circular dependencies and tightly coupled test packages.

All packages should have some unit tests and possibly integration tests, but the degree to which they should have them is based on the package's category (described later).


### Test Coverage

Some packages should have a mechanism setup to capture test coverage information (if applicable to the language).
Coverage tools exist for some of the languages described here including C, C++, and Python, but possibly others.
When possible coverage should be measured in terms of branch coverage, as opposed to statement or function coverage.


## Versioning

*(Planned; not yet used)*

We will use the Semantic Versioning guidelines for versioning:

http://semver.org/

Anything below version `1.0.0` is free to make changes at will and for most of our near-term development this will be the case.
In general though for versions less than `1.0.0` we should increment the `minor` (as `major.minor.patch`) when we break existing API and increment `patch` for anything else.

Another part of adhering to the Semantic Versioning guidelines is that every package must declare a public API.
The declaration for most C and C++ packages is simple, it is any header that it installs, but it is acceptable to define a set of symbols which are considered private.
When ever possible having private symbols in public headers should be avoided.
For other languages like Python, a public API must be explicitly defined, so that it is clear what symbols can be relied on with respect to the versioning guidelines.
The public API can also be extended to build artifacts like configuration variables, CMake config files, etc. as well as executables and command line options and output.
Any elements of the public API should be clearly stated in the package's documentation.
If something you are using is not explicitly listed as part of the public API in the package's documentation, then you cannot depend on it not changing between minor or patch versions.

With respect to library versioning, we will version all libraries within a package together.
This means that libraries inherit their version from the package.
This keeps library and package versions from diverging and shares reasoning with the policy of releasing packages which share a repository together.
If you need libraries to have different versions then consider splitting them into different packages.


### Filesystem Layout

The filesystem layout of packages and repositories should follow the same conventions in order to provide a consistent experience for users browsing our source code.


#### Package layout

- `src`: contains all C and C++ code
 - Also contains C/C++ headers which are not installed
- `include`: contains all C and C++ headers which are installed
 - `<package name>`: for all C and C++ installed headers they should be folder namespaced by the package name
- `<package_name>`: contains all Python code
- `test`: contains all automated tests and test data
- `doc`: contains all the documentation
- `package.xml`: as defined by [REP-0140](http://www.ros.org/reps/rep-0140.html) (may be updated for prototyping)
- `CMakeLists.txt`: only ROS packages which use CMake
- `setup.py`: only ROS packages which use Python code only
- `README.md`: README which can be rendered on Github as a landing page for the project
 - This can be as short or detailed as is convenient, but it should atleast link to project documentation
 - Consider putting a CI or code coverage tag in this readme
 - It can also be `.rst` or anything else that Github supports
- `LICENSE`: A copy of the license or licenses for this package
- `CHANGELOG.rst`: [REP-0132](http://www.ros.org/reps/rep-0132.html) compliant changelog


#### Repository layout

Each package should be in a subfolder which has the same name as the package.
If a repository contains only a single package it can optionally be in the root of the repository.

The root of the repository should have a `CONTRIBUTING.md` file describing the contribution guidelines.
This might include license implication when using e.g. the Apache 2 License.


## Documentation

*(API docs are not yet being automatically generated)*

All packages should have these documentation elements:

- Description and purpose
- Definition and description of the public API
- Examples
- How to build and install (should reference external tools/workflows)
- How to build and run tests
- How to build documentation
- How to develop (useful for describing things like `python setup.py develop`)

Each package should describe itself and its purpose or how it is used in the larger scope.
The description should be written, as much as possible, assuming that the reader has stumbled onto it without previous knowledge of ROS or other related projects.

Each package should define and describe its public API so that there is a reasonable expectation for users what is covered by the semantic versioning policy.
Even in C and C++, where the public API can be enforced by API and ABI checking, it is a good opportunity to describe the layout of the code and the function of each part of the code.

It should be easy to take any package and from that package's documentation understand how to build, run, build and run tests, and build the documentation.
Obviously we should avoid repeating ourselves for common workflows, like build a package in a workspace, but the basic workflows should be either described or referenced.

Finally, it should include any documentation for developers.
This might include workflows for testing the code using something like `python setup.py develop`, or it might mean describing how to make use of extension points provided by you package.

Examples:

- capabilities: http://docs.ros.org/hydro/api/capabilities/html/
 - This one gives an example of docs which describe the public API
- catkin_tools: https://catkin-tools.readthedocs.org/en/latest/development/extending_the_catkin_command.html
 - This is an example of describing an extension point for a package


### Best Practices

TBD


## Package Categories

*(Planned; not yet used)*

The policies will apply differently to packages depending on their categorization.
The categories are meant to give some expectation as to the quality of a package and allows us to be more strict or compliant with some packages and less so with others.


### (Level 1)

This category should be used for packages which are required for a reasonable ROS system in a production environment.
That is to say that after you remove development tools, build tools, and introspection tools, these packages are still left over as requirements for a basic ROS system to run.
However, just because you can conceive a system which does not need a particular package does not mean that it shouldn't be called 'Level 1', in fact the opposite is true.
If we can imagine that any reasonable production scenario where a package would be used in some essential function, then that package should be considered for this category.
However, packages which we consider essential to getting a robot up and running quickly, but is a generic solution to the problem should probably not start out as 'Level 1'.

For Example, the packages which provide in-process communication, interprocess communication, generated message runtime code, and component lifecycle should probably all be considered 'Level 1'.
However, a package which provides pose estimation (like `robot_pose_ekf`) is a generic solution something that most people need, but is often replaced with a domain specific solution in production, and therefore it should probably not start out as 'Level 1'.
However, it may upgrade to it at a later date, if it proves to be a solution that people want to use in their products.

Tools, like `rostopic`, generally do not fall into this category, but are not categorically excluded.
For example, it may be the case the tool which launches and verifies a ROS graph (something like `roslaunch`) may need to be considered 'Level 1' for use in production systems.


#### Package Requirements

Requirements to be considered a 'Level 1' package:

- Have a strictly declared public API
- Have API documentation coverage for public symbols
- Have 100 percent branch code coverage from unit and integration tests
- Have system tests which cover any scenarios covered in documentation
- Have system tests for any corner cases encountered during testing
- Must be >= version 1.0.0


#### Change Control Process

The change control process requires all changes, regardless of trivialness, must go through a pull request.
This is to ensure a complete memoranda of changes to the code base.
In order for a pull request to get merged:

- Changes must be reviewed by two reviewers
- Commits must be concise and descriptive
- All automated tests must be run in CI on all applicable platforms (Windows, versions of Linux, OS X, ARM)
- Code coverage must stay at 100 percent
- Any changes which require updates to documentation must be made before merging


### (Level 2)

These are packages which need to be solidly developed and might be used in production environments, but are not strictly required, or are commonly replaced by custom solutions.
This can also include packages which are not yet up to 'Level 1' but intend to be in the future.


### (Level 3)

These are packages which are useful for development purposes or introspection, but are not recommended for use in embedded products or mission critical scenarios.
These packages are more lax on documentation, testing, and scope of public API's in order to make development time lower or foster addition of new features.


### (Level 4)

These are demos, tutorials, or experiments.
They don't have strict requirements, but are not excluded from having good documentation or tests.
For example, this might be a tutorial package which is not intended for reuse but has excellent documentation because it serves primarily as an example to others.
