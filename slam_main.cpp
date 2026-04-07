#include "util/bal.h"
#include "viz/vizlib_bal.h"

int main(int argc, char **argv) {

  const std::string problem_path = "../problem-16-22106-pre.txt";
  util::BALProblem bal_problem(problem_path);

  viz::pangolin_draw(bal_problem);
  return 0;
}
