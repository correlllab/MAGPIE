import asyncio

async def _read_stream( stream, cb ):  
    """ Process a stream until there isn't any data """
    while True:
        line = await stream.readline()
        if line:
            cb(line)
        else:
            break

async def _stream_subprocess( cmd, stdout_cb, stderr_cb ):  
    """ Start a subprocess and stream its output """

    # Create the subprocess
    process = await asyncio.create_subprocess_exec(
        *cmd,
        stdout = asyncio.subprocess.PIPE, 
        stderr = asyncio.subprocess.PIPE
    )

    # Returns a Future instance, allowing high level grouping of tasks
    await asyncio.gather(
        _read_stream( process.stdout, stdout_cb ),
        _read_stream( process.stderr, stderr_cb )
    )

    # Supports waiting to be stopped after the first task is done, 
    # or after a specified timeout, allowing lower level precision of operations
    return await process.wait()


def execute( cmd, stdout_cb, stderr_cb ):  
    """ Run a command, and monitor its output in real time """
    
    # Start an event loop
    loop = asyncio.get_event_loop()
    
    # Loop until the process completes
    rc = loop.run_until_complete(
        # Start a streaming subprocess
        _stream_subprocess(
            cmd,
            stdout_cb,
            stderr_cb,
        )
    )
    loop.close()
    return rc

if __name__ == '__main__':  
    print(execute(
        ["bash", "-c", "echo stdout && sleep 1 && echo stderr 1>&2 && sleep 1 && echo done"],
        lambda x: print("STDOUT: %s" % x),
        lambda x: print("STDERR: %s" % x),
    ))