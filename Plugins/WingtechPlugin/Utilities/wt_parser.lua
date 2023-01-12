-- trivial protocol example
-- declare our protocol
trivial_proto = Proto("Wingtech","Wingtech")
-- create a function to dissect it
function trivial_proto.dissector(buffer,pinfo,tree)
	pinfo.cols.protocol = "Wingtech"
	local subtree = tree:add(trivial_proto,buffer(),"Wingtech data")

	local curr = 0


	local headerSize = 12

	local blockPerPacket = 2

        local nbLaser = 128
	local laserSize = 3
        local blockSize =  2 + nbLaser * laserSize  

        local tailSize = 4 + 17 + 26

	-- Packet Header --
	local header_subtree = subtree:add(buffer(curr,headerSize),"Header")


        local Sob = buffer(curr, 2):uint()
	header_subtree:add(buffer(curr,2),"Sob : " .. Sob)
	curr = curr + 2

	local VersionMajor = buffer(curr, 1):uint()
	header_subtree:add(buffer(curr,1),"VersionMajor : " .. VersionMajor)
	curr = curr + 1

	local VersionMinor = buffer(curr, 1):uint()
	header_subtree:add(buffer(curr,1),"VersionMinor : " .. VersionMinor)
	curr = curr + 1

	local DistUnit = buffer(curr, 1):uint()
	header_subtree:add(buffer(curr,1),"DistUnit : " .. DistUnit)
	curr = curr + 1

        local Flags = buffer(curr, 1):uint()
	header_subtree:add(buffer(curr,1),"Flags : " .. Flags)
	curr = curr + 1

	local LaserNum = buffer(curr, 1):uint()
	header_subtree:add(buffer(curr,1),"LaserNum : " .. LaserNum)
	curr = curr + 1

	local BlockNum = buffer(curr, 1):uint()
	header_subtree:add(buffer(curr,1),"BlockNum : " .. BlockNum)
	curr = curr + 1

	local EchoCount = buffer(curr, 1):uint()
	header_subtree:add(buffer(curr,1),"EchoCount : " .. EchoCount)
	curr = curr + 1

        local EchoNum = buffer(curr, 1):uint()
	header_subtree:add(buffer(curr,1),"EchoNum : " .. EchoNum)
	curr = curr + 1

	local Reserved = buffer(curr, 2):uint()
	header_subtree:add(buffer(curr,2),"Reserved : " .. Reserved)
	curr = curr + 2


	---- bock Return ----
        local size = blockPerPacket * blockSize
	local blockreturns = subtree:add(buffer(curr,size),"Blocks")

	for i=0, blockPerPacket-1
	do
		local block_subtree = blockreturns:add(buffer(curr,blockSize),"Block Return : " ..i)

		local Azimuth = buffer(curr, 2):uint()
		block_subtree:add(buffer(curr,2),"Azimuth : " .. Azimuth)
		curr = curr + 2

	        local all_laser_subtree = block_subtree:add(buffer(curr,laserSize * nbLaser),"All Lasers Return")

		for j=0, nbLaser-1
        	do
		        local laser_subtree = all_laser_subtree:add(buffer(curr,laserSize),"Laser Return : " ..j)

			local Distance = buffer(curr,2):uint()
			laser_subtree:add(buffer(curr,2),"Distance  : " .. Distance)
			curr = curr + 2

			local Intensity = buffer(curr,1):uint()
			laser_subtree:add(buffer(curr,1),"Intensity  : " .. Intensity)
			curr = curr + 1
		end

	end


	-- Tail --
	local tail_subtree = subtree:add(buffer(curr,tailSize),"Tail")


        local CRC = buffer(curr, 4):uint()
	tail_subtree:add(buffer(curr,4),"CRC : " .. CRC)
	curr = curr + 4

	for n=0, 16
        do
		local functionSafety_subtree = tail_subtree:add(buffer(curr,1),"functionSafety subtree : " ..n)

		local functionSafety = buffer(curr,1):uint()
		functionSafety_subtree:add(buffer(curr,1),"functionSafety  : " .. functionSafety)
		curr = curr + 1
	end

	local Reserved1 = buffer(curr, 3):uint()
	tail_subtree:add(buffer(curr,3),"Reserved1 : " .. Reserved1)
	curr = curr + 3

	local Reserved21 = buffer(curr, 1):uint()
	tail_subtree:add(buffer(curr,1),"Reserved21 : " .. Reserved21)
	curr = curr + 1

	local Reserved22 = buffer(curr, 1):uint()
	tail_subtree:add(buffer(curr,1),"Reserved22 : " .. Reserved22)
	curr = curr + 1

	local Reserved23 = buffer(curr, 1):uint()
	tail_subtree:add(buffer(curr,1),"Reserved23 : " .. Reserved23)
	curr = curr + 1

	local Padding1 = buffer(curr,2):uint()
	tail_subtree:add(buffer(curr,2),"Padding1  : " .. Padding1)
	curr = curr + 2

	local ShutdownFlag = buffer(curr,1):uint()
	tail_subtree:add(buffer(curr,1),"ShutdownFlag  : " .. ShutdownFlag)
	curr = curr + 1

	local ReturnMode = buffer(curr, 1):uint()
	tail_subtree:add(buffer(curr,1),"ReturnMode : " .. ReturnMode)
	curr = curr + 1

	local MotorSpeed = buffer(curr,2):uint()
	tail_subtree:add(buffer(curr,2),"MotorSpeed  : " .. MotorSpeed)
	curr = curr + 2

	local UTCTime1 = buffer(curr, 2):uint()
	tail_subtree:add(buffer(curr,2),"UTCTime1 : " .. UTCTime1)
	curr = curr + 2

	local UTCTime2 = buffer(curr, 4):uint()
	tail_subtree:add(buffer(curr,4),"UTCTime2 : " .. UTCTime2)
	curr = curr + 4

	local Timestamp = buffer(curr,4):uint()
	tail_subtree:add(buffer(curr,4),"Timestamp  : " .. Timestamp)
	curr = curr + 4

	local SeqNum = buffer(curr,4):uint()
	tail_subtree:add(buffer(curr,4),"SeqNum  : " .. SeqNum)
	curr = curr + 4
end


-- load the udp.port table
udp_table = DissectorTable.get("udp.port")
-- register our protocol to handle udp port 7777
udp_table:add(2368,trivial_proto)
