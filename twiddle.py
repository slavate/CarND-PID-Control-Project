import subprocess

def run_pid(p, learn):
    ## prepare arguments for pid.exe
    args = r".\build\Debug\pid.exe "
    ## timestep for simulator reset, 2500
    time2reset = 3000
    ## max error
    max_error = 10
    ## max speed
    max_speed = "120"
    ## pid parameters Kp, Kd, Ki
    for i in range(len(p)):
        args += str(p[i]) + " "
    ## max speed for simulator
    args += max_speed


    ## Open Process, http://blog.endpoint.com/2015/01/getting-realtime-output-using-python.html
    process = subprocess.Popen(args, stdout=subprocess.PIPE)
    ## Hear for output from the process
    while True:
        output = process.stdout.readline().decode()
        if output == '' and process.poll() is not None:
            break
        if output:
            oList = output.strip().split(";")
            if len(oList) > 1:
                timestep = int(oList[0])
                pid_err = float(oList[1])
                #print("#" + oList[0] + ": " + str(pid_err))
                if learn:
                    if timestep >= time2reset or pid_err >= max_error:
                        process.kill()
                        break
        rc = process.poll()
    return pid_err


# From Lesson 16 PID Control, Chapter 14 Parameter Optimization
# Make this tolerance bigger if you are timing out!
def twiddle(tol=0.2):
    # Don't forget to call `make_robot` before you call `run`!
    params = [0.2, 10.0, 0.1]
    dp = [0.1, 1.0, 0.1]
    best_err = run_pid(params, True)
    # twiddle loop here
    iteration = 0
    print("#", iteration, params, "->", best_err)
    while sum(dp) > tol:
        for i in range(len(params)):
            params[i] += dp[i]
            err = run_pid(params, True)
            print("params:", params, "->", err)
            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                params[i] -= 2 * dp[i]
                err = run_pid(params, True)
                print("params:", params, "->", err)
                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    params[i] += dp[i]
                    dp[i] *= 0.9
        iteration += 1
        print("#", iteration, params, dp, "->", err)

    return params, best_err

run_pid([0.05, 20.0, 0.1], False)
#run_pid([0.1, 10.0, 0.0006], False)
# p, best_err = twiddle()
# print("############################################################################")
# print("best coefficients: ", p)

