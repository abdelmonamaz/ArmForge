#pragma once

namespace af {

// Denavit–Hartenberg parameters for one joint frame.
// Convention : modified DH (Craig).
//   a     : link length   — translation along x_{i-1}
//   alpha : link twist    — rotation about x_{i-1}
//   d     : joint offset  — translation along z_i
//   theta : joint angle offset, added to the variable q_i
// Units : metres and radians throughout AF_Core.
struct DHParameters {
    double a;
    double alpha;
    double d;
    double theta;
};

} // namespace af
