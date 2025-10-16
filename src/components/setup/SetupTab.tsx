import React from "react";
import RTKInjectorPanel from "./RTKInjectorPanel";
import BasicSetupPanel from "./BasicSetupPanel";

const SetupTab: React.FC = () => {
  return (
    <div className="p-6 text-white bg-slate-900 min-h-screen">
      <h1 className="text-2xl font-bold mb-4">⚙️ Setup Panel</h1>
      <p className="text-gray-300 mb-6">
        Configure rover-related features and start RTK correction streams here.
      </p>
      {/* Basic Setup Section */}
      <BasicSetupPanel /> 
      {/* RTK Injector Section */}
      <RTKInjectorPanel />
    </div>
  );
};

export default SetupTab;
