std_msgs/Header header
string epoch
uint64 revision
string consumer
# Derived publish of applied. Publishers must set this equal to applied.
# Receivers ignore this field; applied is the only document-apply authority.
bool success
string message
# Document of this epoch/revision is in the consumer. Explicit; false is not applied.
bool applied
# The consumer can perform its operation on this version. Distinct from applied.
bool operational
# "" | "ok" | "unsupported". unsupported is a declared capability gap, not a retryable sync miss.
string capability
# Consumer process generation. Zero means unspecified; a lower generation is stale.
uint32 generation
