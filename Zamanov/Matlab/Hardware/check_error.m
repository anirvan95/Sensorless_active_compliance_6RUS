function [net_result] = check_error(port_number)

    PROTOCOL_VERSION = 1;
    COMM_SUCCESS = 0;  % Communication Success result value
    COMM_TX_FAIL = -1001;  % Communication Tx Failed

    dxl_comm_result = getLastTxRxResult(port_number, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_number, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
    net_result = dxl_comm_result+dxl_error;
end

