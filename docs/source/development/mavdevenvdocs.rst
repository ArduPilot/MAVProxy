Updating MAVProxy Documentation
=================================

Use git to download the MAVProxy Documentation source:

.. code:: bash

    git clone -b gh-pages http://github.com/ardupilot/mavproxy

Install sphinx-docs

Linux

.. code:: bash

    sudo apt-get install python-sphinx

macOS

.. code:: bash

    brew install sphinx-doc
    brew link sphinx-doc --force

After making the desired changes, Build the docs using following calls:

.. code:: bash

    cd docs
    ./sphinxLinux.sh

Make Pull-Requests via github to http://github.com/ardupilot/mavproxy gh-pages branch for updates.
