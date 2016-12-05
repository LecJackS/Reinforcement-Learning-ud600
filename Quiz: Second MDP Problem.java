import burlap.behavior.singleagent.Policy;
import burlap.behavior.singleagent.planning.commonpolicies.GreedyQPolicy;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.statehashing.DiscreteStateHashFactory;
import burlap.domain.singleagent.graphdefined.GraphDefinedDomain;
import burlap.oomdp.auxiliary.DomainGenerator;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.core.*;
import burlap.oomdp.singleagent.GroundedAction;
import burlap.oomdp.singleagent.RewardFunction;

public class SecondMDP {
    
    DomainGenerator				dg;
    Domain						domain;
    State						initState;
    RewardFunction				rf;
    TerminalFunction			tf;
    DiscreteStateHashFactory	hashFactory;

    public SecondMDP(double p1, double p2) {
        int numStates = 6;
        this.dg = new GraphDefinedDomain(numStates);
        //double p1 = p1;
        //double p2 = p2;
        
        //actions from initial state 0
        ((GraphDefinedDomain) this.dg).setTransition(0, 0, 1, 1.-p1);
        ((GraphDefinedDomain) this.dg).setTransition(0, 0, 0, p1);
        ((GraphDefinedDomain) this.dg).setTransition(0, 1, 2, 1.);
        
		//transitions from state 1
		((GraphDefinedDomain) this.dg).setTransition(1, 0, 3, 1.-p2);
		((GraphDefinedDomain) this.dg).setTransition(1, 0, 5, p2);
		((GraphDefinedDomain) this.dg).setTransition(1, 1, 4, 1.);

		//transitions from state 3
		((GraphDefinedDomain) this.dg).setTransition(3, 0, 1, 1.);

		//transitions from state 4
		((GraphDefinedDomain) this.dg).setTransition(4, 0, 5, 1.);
		
		//transitions from state 2
		((GraphDefinedDomain) this.dg).setTransition(2, 0, 1, 1.);


        this.domain = this.dg.generateDomain();
        this.initState = GraphDefinedDomain.getState(this.domain,0);
		this.rf = new SpecificRF();
        this.tf = new SingleStateTF(5);
        this.hashFactory = new DiscreteStateHashFactory();
    }
    
    public static class SpecificRF implements RewardFunction {
		
		@Override
		public double reward(State s, GroundedAction a, State sprime) { 
			int sid = GraphDefinedDomain.getNodeId(s);
			int sprimeid = GraphDefinedDomain.getNodeId(sprime);
			double r=0;
			
			if( sid == 0 ) { // initial state
			    if( sprimeid == 0 ) {
				    r = -1;
			    }
			    else if( sprimeid == 1 ) {
				    r = +3;
			    }
			    else if( sprimeid == 2 ) {
				    r = +1;
			    }
			    
			}
			else if( sid == 1 ) { 
				if( sprimeid == 3 ) {
				    r = +1;
			    }
			    else if( sprimeid == 5 ) {
				    r = 0;
			    }
			    else if( sprimeid == 4 ) {
				    r = +2;
			    }
			}
			else if( sid == 2 ) {
				r = 0;
			}
			else if( sid == 3 ) {
				r = 0;
			}
			else if( sid == 4 ) {
				r = 0;
			}
			else if( sid == 5 ) {
				r = 0;
			}
			else {
				throw new RuntimeException("Unknown state: " + sid);
			}
			
			return r;
		}
    }
    
    public static class SingleStateTF implements TerminalFunction {
        int terminalSid;
        
        public SingleStateTF(int sid){
            this.terminalSid = sid;
        }
        
        @Override
        public boolean isTerminal(State s) {
            int sid = GraphDefinedDomain.getNodeId(s);
            return sid == this.terminalSid;
        }
    }
    
    private ValueIteration computeValue(double gamma) {
    	double maxDelta = 0.0001;
    	int maxIterations = 1000;
    	ValueIteration vi = new ValueIteration(this.domain, this.rf, this.tf, gamma, 
    			this.hashFactory, maxDelta, maxIterations);
    	vi.planFromState(this.initState);
    	return vi;
    }
    
	public String bestActions(double gamma) {
        // Return one of the following Strings
        // "a,c"
        // "a,d"
        // "b,c" 
        // "b,d"
        // based on the optimal actions at states S0 and S1. If 
        // there is a tie, break it in favor of the letter earlier in
        // the alphabet (so if "a,c" and "a,d" would both be optimal, 
        // return "a,c").
        ValueIteration vi = this.computeValue(gamma);
		
		Policy p = new GreedyQPolicy(vi);
		State s0 = GraphDefinedDomain.getState(this.domain, 0);
		State s1 = GraphDefinedDomain.getState(this.domain, 1);
		
		String s0Action = p.getAction(s0).actionName().replaceAll("action0", "a").replaceAll("action1", "b");
		String s1Action = p.getAction(s1).actionName().replaceAll("action0", "c").replaceAll("action1", "d");
		
		String policy = s0Action + "," + s1Action;
		return policy;
    }
	
	public static void main(String[] args) {
		double p1 = 0.5;
		double p2 = 0.5;
		SecondMDP mdp = new SecondMDP(p1,p2);
		
		double gamma = 0.5;
		System.out.println("Best actions: " + mdp.bestActions(gamma));
	}
}