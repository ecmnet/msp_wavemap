Contributing
############
.. highlight:: bash
.. rstcheck: ignore-roles=gh_file

Thank you for your interest in contributing to wavemap!

Questions
*********
If you have any questions, feel free to ask them in the `Q&A section <https://github.com/ethz-asl/wavemap/discussions/categories/q-a>`_ of our GitHub Discussions.

Before posting, please check if your question has already been addressed in our :doc:`installation <installation/index>` or :doc:`usage <tutorials/index>` tutorials. We're happy to answer any remaining theoretical or code-related questions, and help you optimize wavemap for your specific sensor setup.

Bug reports & Feature requests
******************************
We encourage you to submit bug reports and feature requests. You can do so using the relevant GitHub Issue templates:

* `Bug report <https://github.com/ethz-asl/wavemap/issues/new?template=bug_report.md>`_
* `Feature request <https://github.com/ethz-asl/wavemap/issues/new?template=feature_request.md>`_

In addition to requests for new functionality, do not hesitate to open feature requests for:

* API methods and interfaces
* Extensions to the documentation and examples

Pull requests
*************
We encourage `pull requests <https://github.com/ethz-asl/wavemap/pulls>`_, especially for fixes and improvements to the documentation, code, and ROS launch and config files. Also feel free to share launch and config files for new sensor setups and applications.

If you would like to contribute a new feature, please open a corresponding `feature request <https://github.com/ethz-asl/wavemap/issues/new?template=feature_request.md>`_ before you start and mention that you are willing to help with its development. This way, we can synchronize and avoid duplicated efforts.

Coding standards
****************
This project follows the `Google Style Guide <https://google.github.io/styleguide/cppguide.html>`_ for C++.

To maintain code quality, we use the `pre-commit <https://pre-commit.com/>`_ framework to automatically format, lint, and perform basic code checks. You can install pre-commit together with the dependencies required to run all of wavemap's checks with::

    cd <path_to_wavemap_repo>
    ./tooling/scripts/install_pre_commit.sh

After running the above script, pre-commit will automatically check changed code when it is committed to git. All the checks can also be run manually at any time by calling::

    # cd ~/catkin_ws/src/wavemap
    pre-commit run --all

In case you really need to commit some changes that are not accepted by pre-commit, you can use git's ``--no-verify`` flag as::

    git commit --no-verify -m "Your commit message"

Testing
*******
Wavemap's codebase includes a broad suite of tests. These are run in our Continuous Integration pipeline for active merge requests, `see here <https://github.com/ethz-asl/wavemap/actions/workflows/ci.yml>`_. You can also run the tests locally with::

    cd <path_to_wavemap_repo>
    ./tooling/scripts/build_and_test_all.sh

The tests are located in the `test` subfolders of each package and implemented using the `GoogleTest <http://google.github.io/googletest/>`_ framework.

Documentation
*************
The documentation consists of pages for specific topics, located in the :gh_file:`docs/pages <docs/pages>` directory, and inline Doxygen annotations for the C++ source code.

To read the documentation for a specific wavemap version:

Latest release
==============
Browse to https://ethz-asl.github.io/wavemap.

Specific release
================
Find the release in question under `GitHub Releases <https://github.com/ethz-asl/wavemap/releases>`_. Under `Assets`, at the bottom of the release's description, click and download `docs.tar`. Then unpack the archive into an empty folder and double-click index.html to open it in a browser.

Local version
=============
In a terminal, run::

    cd <path_to_wavemap_repo>
    ./tooling/scripts/preview_docs.sh

This will build a preview of the documentation based on your local wavemap version and changes and open the result in a web browser.
