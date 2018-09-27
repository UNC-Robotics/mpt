Running the test requires either gnumake, or a recent version of bash and the ninja build system.

To use gnumake:

    % make -j8

To use ninja:

    % ./generate_ninja_build.sh
    % ninja

The command ./generate_ninja_build.sh will have to be run every time a new class is added.
