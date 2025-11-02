import React from "react";
import RTKInjectorPanel from "./RTKInjectorPanel";
// import BasicSetupPanel from "./BasicSetupPanel";

const SetupTab: React.FC = () => {
  return (
    <div className="w-full h-full overflow-y-auto custom-scrollbar p-3 text-white bg-slate-900">
      <h1 className="text-xl font-bold mb-3">📡 RTK Setup</h1>
      <p className="text-gray-300 text-sm mb-4">
        Configure RTK correction streams and related features here.
      </p>
      {/* RTK Injector Section */}
      <RTKInjectorPanel />
    </div>
  );
};

export default SetupTab;
