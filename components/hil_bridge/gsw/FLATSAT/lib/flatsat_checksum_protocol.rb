# COSMOS write protocol for the HITL FlatSat interfaces: fills in the CCSDS command checksum.
#
# The checksum byte (offset 7) is chosen so the XOR of every byte in the packet is 0xFF, the cFS
# convention the FlatSat OBC enforces (ICD 3.2). NOS3's cFS doesn't check it, so its targets send 0.
require 'cosmos/interfaces/protocols/protocol'

module Cosmos
  class FlatsatChecksumProtocol < Protocol
    CHECKSUM_OFFSET = 7

    def write_data(data)
      if data.length > CHECKSUM_OFFSET
        data = data.dup
        data.setbyte(CHECKSUM_OFFSET, 0)
        x = 0
        data.each_byte { |b| x ^= b }
        data.setbyte(CHECKSUM_OFFSET, x ^ 0xFF)
      end
      super(data)
    end
  end
end
