import os
import stream_tee as stream_tee
import __main__ as main

def experiment_name():
    experiment = os.path.splitext(os.path.split(main.__file__)[1])[0]
    name = experiment + '_' + stream_tee.generate_timestamp()
    return name

