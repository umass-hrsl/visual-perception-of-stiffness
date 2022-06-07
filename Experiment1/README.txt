README:
This folder contains all the pertainent information related to Experiment 1.
getTwo_LinkStates_Experiment1.m:
	This function is used to create the structures of the simulated arm motions that were shown to subjects in all four conditions of Experiment 1.
	The structures for the Original, Constant, Reverse, and Variable conditions are saved as twoLinkStates_Experiment1_Original.mat, twoLinkStates_Experiment1_Constant.mat, twoLinkStates_Experiment1_Reverse.mat, and twoLinkStates_Experiment1_Variable.mat, respectively, in their respective folders.
subfolders are Original, Constant, Reverse, and Variable:
	They contain the respective subject data and 
	The structure of simulated arm motions that were shown to subjects in Experiment 1, produced from getTwo_LinkStates_Experiment1.m, for each condition
runTwoLinkExperiment_Experiment2:
	This function sets up and runs the GUI in which subjects observed the simulated arm motions and made their stiffness estimates.
	It uses twoLinkStates_Experiment1_Original.mat, twoLinkStates_Experiment1_Constant.mat, twoLinkStates_Experiment1_Reverse.mat, or twoLinkStates_Experiment1_Variable.mat to show the simulated arm motions.
	At the end of each subjects' session, it saves their stiffness estimates in a data file based on the user established subject de-identified "initials"
S__.mat:
	Each mat file named S__.mat, where the __ represents a subject number, is the data for a given subject.
	The mat file loads two variables: ratings and trialConditions.
	The ratings are the subject's stiffness estimates ranging from (1 to 7).
	The trialConditions are the codified trial conditions:
		0 denotes elbow stiffness = 0 Nm/rad
		1 denotes elbow stiffness = 10 Nm/rad
		2 denotes elbow stiffness = 20 Nm/rad
		3 denotes elbow stiffness = 30 Nm/rad
		4 denotes elbow stiffness = 40 Nm/rad
		5 denotes elbow stiffness = 50 Nm/rad
plotGroupResults.m:
	This code shows at all the subject data in Experiment 2.
	This code helped produce Figure 6.
subjectCodifiedResponses.mat:
	This mat file returns a variable with all the codified subject responses.
	Each row is a different subject.
	column 1: identifies the condition / experiement
		1 denotes Experiment 1 Constant Condition
		2 denotes Experiment 1 Constant Reverse
		3 denotes Experiment 1 Constant Variable
		4 denotes Experiment 2
	column 2: denotes whether endpoint and/or joint information was used (0 for neither, 1 for joint, 2 for endpoint, 3 for both)
	column 3: denotes whether  path and/or trajectory information was used (0 for neither, 1 for path, 2 for trajectory, 3 for both)
	column 4: denotes mention of "jerk" or "smooth" (0 for neither, 1 for jerk/smooth)
	column 5: lists the R^2 for subjects who participated in Experiment 1