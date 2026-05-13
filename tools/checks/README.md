# Checks

Place read-only readiness and contract checks here.

Rules:
- Must not command robot motion.
- Must not call RG2 open/close/set-width unless the filename and README say it
  is an explicit hardware check.
- Should return non-zero on failed readiness.
- Should print the exact service/topic/action names checked.

Root-level `check_*.sh` files are being kept temporarily for compatibility.
