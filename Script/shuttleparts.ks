//		THIS IS A TESTING SCRIPT PLEASE DON'T USE

clearscreen.


local shuttleparts IS getShuttleParts().

local stackmass is 0.
for p in shuttleparts {
	set stackmass to stackmass + p:mass.
}

print "shuttle mass is " + stackmass.




//returns the list of parts making up the Orbiter and External Tank
function getShuttleParts {

	function removeChildrenPartsRecursively {
		parameter partslist.
		parameter part.
		
		local partchildren is part:children:copy.
		
		if partchildren:length > 0 {
			for p in partchildren {
			
				removeChildrenPartsRecursively(
					partslist,
					p 
				).
			
			}
		}
		
		partslist:remove(partslist:find(part)).
		return.

	}
	
	
	local et is ship:partsdubbed("ShuttleExtTank")[0].

	local shuttleParts is ship:parts:copy.

	removeChildrenPartsRecursively(
		shuttleParts,
		et
	).

	shuttleParts:add(et).
	
	return shuttleParts.
}


