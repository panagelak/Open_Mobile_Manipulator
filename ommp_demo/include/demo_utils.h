#ifndef DEMO_UTILS_H
#define DEMO_UTILS_H

static bool list_actions;



static int first = -1, last = -1, execution_result = -2;

void mySpin()
{
  while (execution_result != 0 && execution_result != -1)
  {
    ros::Duration(0.001).sleep();
    ros::spinOnce();
  }
}

std::vector<double> deg2rad(const std::vector<double> deg_vec)
{
  std::vector<double> rad_vec;
  for (double elem : deg_vec)
    rad_vec.push_back(elem * M_PI / 180);
  return rad_vec;
}

void setJointGoalValues(integration::MoveToJointsGoal &goal, const std::vector<double> values)
{
  for (uint i = 0; i < values.size(); i++)
    goal.joints[i].value = values[i];
}

void setCartesianGoalValues(integration::MoveToPoseGoal &goal, const std::vector<double> values)
{
  goal.target_pose.pose.position.x = values[0];
  goal.target_pose.pose.position.y = values[1];
  goal.target_pose.pose.position.z = values[2];
  goal.target_pose.pose.orientation.x = values[3];
  goal.target_pose.pose.orientation.y = values[4];
  goal.target_pose.pose.orientation.z = values[5];
  goal.target_pose.pose.orientation.w = values[6];
}

template <typename M>
void printGoaommpg(std::ofstream &file, const M goal, std::string name, int number)
{
  std::ofstream temp_ofile("temp.yaml");
  temp_ofile << goal;
  temp_ofile.close();

  std::ifstream temp_ifile("temp.yaml");
  std::string goal_str((std::istreambuf_iterator<char>(temp_ifile)), std::istreambuf_iterator<char>());

  uint i = 0;
  file << "  id: " << number << "\n";
  file << "  description: " << name << "\n  ";
  for (char &c : goal_str)
  {
    file << c;
    if (c == '\n')
      file << "  ";

    if (c == '[' && goal_str[i + 1] == ']')
      goal_str.insert(i + 2, ":");

    i++;
  }
  file << std::endl << std::endl;
}

template <typename M>
bool waitNcheck(const M client)
{
  client->waitForResult();
  if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    return true;
  ROS_ERROR("Goal FAILED. Sequence terminated");
  return false;
}

template <typename M1, typename M2>
bool performAction(const uint8_t id, const std::string info, M1 ac, const M2 goal, bool wait = true)
{
  if (first <= id && last >= id)
  {
    ROS_INFO("[%i] %s", id, info.c_str());
    if (list_actions)
      return true;

    ac->sendGoal(goal);

    uint8_t goal_status = ac->getState().state_;

    while ((goal_status == actionlib::SimpleClientGoalState::PENDING ||
            goal_status == actionlib::SimpleClientGoalState::ACTIVE) &&
           wait == true)
    {
      ros::Duration(0.1).sleep();
      goal_status = ac->getState().state_;
    }

    if (wait)
    {
      switch (goal_status)
      {
        case actionlib::SimpleClientGoalState::SUCCEEDED:
          return true;
        // TODO: specify what happens in all the other cases
        default:
          return false;
      }
    }
  }
  return true;
}

template <typename M3, typename M4>
bool callService(const uint8_t id, const std::string info, const M3 sc, M4 *srv, bool force = false)
{
  if ((first <= id && last >= id) || force)
  {
    ROS_INFO("[%i] %s", id, info.c_str());
    if (list_actions)
      return true;

    if (!sc->call(*srv))
    {
      return false;
    }
    else
    {
      return true;
    }
  }
  return true;
}

template <typename M5, typename M6>
bool pubTopic(const uint8_t id, const std::string info, const M5 pub, M6 msg, bool force = false)
{
  if ((first <= id && last >= id) || force)
  {
    ROS_INFO("[%i] %s", id, info.c_str());
    if (list_actions)
      return true;

    pub.publish(msg);
  }
  return true;
}

void printLine()
{
  struct winsize size;
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);
  for (uint i = 0; i < size.ws_col; i++)
    std::cout << "=";
  std::cout << "\n";
}

int terminalWidth()
{
  struct winsize size;
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);
  return size.ws_col;
}

void printTitle(std::string title)
{
  std::cout << std::endl;
  printLine();
  std::string vspace;
  for (uint i = 0; i < (uint(terminalWidth()) - title.length()) / 2 - 3; i++)
    vspace.append("-");
  std::cout << "|" << vspace.c_str() << "| " << title.c_str() << " |" << vspace.c_str() << "|" << std::endl;
  printLine();
  std::cout << std::endl;
}

void printLogo()
{
  // http://patorjk.com/software/taag/#p=display&f=ANSI%20Shadow&t=ommp

  std::string ommp_logo = " ██████╗ ███╗   ███╗███╗   ███╗██████╗ \n"
                          "██╔═══██╗████╗ ████║████╗ ████║██╔══██╗\n"
                          "██║   ██║██╔████╔██║██╔████╔██║██████╔╝\n"
                          "██║   ██║██║╚██╔╝██║██║╚██╔╝██║██╔═══╝ \n"
                          "╚██████╔╝██║ ╚═╝ ██║██║ ╚═╝ ██║██║     \n"
                          " ╚═════╝ ╚═╝     ╚═╝╚═╝     ╚═╝╚═╝     \n ";

  std::string demo_logo = "██████╗ ███████╗███╗   ███╗ ██████╗  \n"
                          "██╔══██╗██╔════╝████╗ ████║██╔═══██╗ \n"
                          "██║  ██║█████╗  ██╔████╔██║██║   ██║ \n"
                          "██║  ██║██╔══╝  ██║╚██╔╝██║██║   ██║ \n"
                          "██████╔╝███████╗██║ ╚═╝ ██║╚██████╔╝ \n"
                          "╚═════╝ ╚══════╝╚═╝     ╚═╝ ╚═════╝  \n";

  std::string vspace_ommp, vspace_s, ommp, ommp_demo;
  int ommp_width = 27, s_width = 66;
  ROS_INFO_STREAM(ommp_width << " " << s_width);
  if (terminalWidth() > ommp_width && terminalWidth() > s_width)
  {
    for (int i = 0; i < (terminalWidth() - ommp_width) / 2; i++)
      vspace_ommp.append(" ");

    for (int i = 0; i < (terminalWidth() - s_width) / 2; i++)
      vspace_s.append(" ");

    ommp = vspace_ommp;
    for (char &c : ommp_logo)
    {
      ommp = ommp + c;
      if (c == '\n')
        ommp = ommp + vspace_ommp;
    }

    ommp_demo = vspace_s;
    for (char &c : demo_logo)
    {
      ommp_demo = ommp_demo + c;
      if (c == '\n')
        ommp_demo = ommp_demo + vspace_s;
    }

    std::cout << std::endl << ommp << std::endl << ommp_demo << std::endl;
  }
}

#endif  // DEMO_UTILS_H
