import asyncio

async def run( cmd ):
    """ Run a subprocess with printouts connected to a pipe """
    proc = await asyncio.create_subprocess_shell(
        cmd,
        stdout = asyncio.subprocess.PIPE,
        stderr = asyncio.subprocess.PIPE
    )

    stdout, stderr = await proc.communicate()

    print(f'[{cmd!r} exited with {proc.returncode}]')
    if stdout:
        print(f'[stdout]\n{stdout.decode()}')
    if stderr:
        print(f'[stderr]\n{stderr.decode()}')

asyncio.run(run('python3.9 dog.py'))

""" ## Result ##
['python3.9 dog.py' exited with 0]
[stdout]
Bark! 
Bark! 
Bark! 
Bark! 
Bark! 
Bark! 
Bark! 
Bark! 
Bark! 
Bark! 
"""