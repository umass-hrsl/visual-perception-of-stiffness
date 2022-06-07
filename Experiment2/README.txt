README:
This folder contains all the pertinent information related to Experiment 2.
getTwo_LinkStates_Experiment2.m:
	This function is used to create a structure of the simulated arm motions that were shown to subjects in Experiment 2.
	The structure is save as twoLinkStates_Experiment2.mat
twoLinkStates_Experiment2.mat:
	The structure of simulated arm motions that were shown to subjects in Experiment 2, produced from getTwo_LinkStates_Experiment2.m
runTwoLinkExperiment_Experiment2:
	This function sets up and runs the GUI in which subjects observed the simulated arm motions and made their stiffness estimates.
	It uses twoLinkStates_Experiment2.mat to show the simulated arm motions.
	At the end of each subjects' session, it saves their stiffness estimates in a data file based on the user established subject de-identified "initials"
S__.mat:
	Each mat file named S__.mat, where the __ represents a subject number, is the data for a given subject.
	The mat file loads two variables: ratings and trialConditions.
	The ratings are the subject's stiffness estimates ranging from (1 to 7).
	The trialConditions are the codified trial conditions:
		1 denotes the Original condition
		2 denotes the Constant condition
		3 denotes the Reverse condition
		4 denotes the Variable condition
kinematicAnalysis_Experiment2.m:
	This code gives a quick look at the varying temporal information provided by the simulations of the Experiment 2.
	This code helped produce Figure 13.
showExperiment2Results.m:
	This code shows at all the subject data in Experiment 2.
	This code helped produce Figure 11.