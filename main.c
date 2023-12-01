#include <stdio.h>
#include <stdlib.h>
#include <processthreadsapi.h>
#include <stdbool.h>
#include ".\src\GeneralPID.h"

double get_cpu_time()
{
  FILETIME a, b, c, d;
  if (GetProcessTimes(GetCurrentProcess(), &a, &b, &c, &d) != 0)
  {
    //  Returns total user time.
    //  Can be tweaked to include kernel times as well.
    return (double)(d.dwLowDateTime |((unsigned long long)d.dwHighDateTime << 32)) * 0.0000001;
  }
  else
  {
    //  Handle error
    return 0;
  }
}

int main()
{
  // Declarecoes iniciais
  GeneralPID pid;
  GeneralPID_init(&pid, 0.00001, 0, 0, 0);
  pid.setLimits(&pid, 1, -1);
  pid.setSetpoint(&pid, 1);
  float measurement = 1;
  int iterations = 1e9;
  // TESTE 1
  // Start measuring time
  double begin = get_cpu_time();
  for (int i = 0; i < iterations; i++)
  {
    pid.execute(&pid, measurement);
  }
  // Stop measuring time and calculate the elapsed time
  double end = get_cpu_time();
  double elapsed = (end - begin);
  printf("Tempo medido 1: %.3f seconds.\n", elapsed);
  // TESTE 2
  //  Start measuring time
  begin = get_cpu_time();
  for (int i = 0; i < iterations; i++)
  {
    pid.execute2(&pid, measurement);
  }
  // Stop measuring time and calculate the elapsed time
  end = get_cpu_time();
  elapsed = (end - begin);
  printf("Tempo medido 2: %.3f seconds.\n", elapsed);

  system("pause");
  return 0;
}