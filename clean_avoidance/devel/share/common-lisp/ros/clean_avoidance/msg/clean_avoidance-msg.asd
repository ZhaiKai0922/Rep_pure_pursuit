
(cl:in-package :asdf)

(defsystem "clean_avoidance-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CleanAvoidanceAction" :depends-on ("_package_CleanAvoidanceAction"))
    (:file "_package_CleanAvoidanceAction" :depends-on ("_package"))
    (:file "CleanAvoidanceActionFeedback" :depends-on ("_package_CleanAvoidanceActionFeedback"))
    (:file "_package_CleanAvoidanceActionFeedback" :depends-on ("_package"))
    (:file "CleanAvoidanceActionGoal" :depends-on ("_package_CleanAvoidanceActionGoal"))
    (:file "_package_CleanAvoidanceActionGoal" :depends-on ("_package"))
    (:file "CleanAvoidanceActionResult" :depends-on ("_package_CleanAvoidanceActionResult"))
    (:file "_package_CleanAvoidanceActionResult" :depends-on ("_package"))
    (:file "CleanAvoidanceFeedback" :depends-on ("_package_CleanAvoidanceFeedback"))
    (:file "_package_CleanAvoidanceFeedback" :depends-on ("_package"))
    (:file "CleanAvoidanceGoal" :depends-on ("_package_CleanAvoidanceGoal"))
    (:file "_package_CleanAvoidanceGoal" :depends-on ("_package"))
    (:file "CleanAvoidanceResult" :depends-on ("_package_CleanAvoidanceResult"))
    (:file "_package_CleanAvoidanceResult" :depends-on ("_package"))
  ))